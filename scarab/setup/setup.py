#!/usr/bin/env python
from __future__ import print_function

import os
import subprocess
import platform
import shutil
import errno
import socket

PACKAGES="""
ccache
ack-grep
openssh-server
ttf-dejavu
libncurses-dev
libnl-dev
libarmadillo-dev
libncurses5-dev
libcgal-dev
htop
emacs

ros-indigo-desktop-full
ros-indigo-openni2-launch
ros-indigo-hokuyo-node
ros-indigo-joystick-drivers
ros-indigo-navigation
ros-indigo-octomap-mapping
ros-indigo-gmapping
ros-indigo-octomap-rviz-plugins

python-rosinstall
python-rosdep

valgrind
gdb
iperf
python-pip
libarmadillo-dev
libcgal-dev
libgsl0-dev libgsl0-dbg libgsl0ldbl
libnl-dev

strace
ack-grep
htop
ipython
screen

python-serial
git-core
build-essential
python-yaml
cmake
openssh-server
minicom

iputils-arping
iputils-tracepath
iputils-clockdiff

terminator

cfengine2
console-common

acpid
ifplugd
batctl
batctl-dbg
traceroute
olsrd
olsrd-plugins
nfs-common
rsync
subversion
rdate
"""

def warning(line):
    print('\033[1;33m' + line + '\033[0m')

def resolve_path(rel_path):
    return os.path.abspath(os.path.expanduser(rel_path))

def symlink(source, dest):
    source = resolve_path(source)
    dest = resolve_path(dest)

    if not os.path.exists(source):
        raise ValueError("Link to non-existant file: %s" % source)

    if os.path.lexists(dest):
        if not (os.path.islink(dest) and source == os.path.realpath(dest)):
            warning("Refusing to link: %s -> %s" % (source, dest))
    else:
        print("Linking: %s -> %s" % (source, dest))
        os.symlink(source, dest)

def exclude(path):
    subprocess.check_call(['dropbox', 'exclude', 'add', resolve_path(path)])

def ainsl(path, line):
    path = resolve_path(path)

    if line[-1] != '\n':
        line += '\n'

    with open(path ,'a+') as f:
        present = any(line == file_line for file_line in f.readlines())
        if not present:
            f.write(line)

def fcopy(abs_path):
    src = './files' + abs_path + '/SCARAB'
    if not os.path.exists(src):
        raise ValueError("fcopy: no file: %s" % src)

    dest = abs_path
    print("Copying: %s -> %s" % (src, dest))
    shutil.copy(src, dest)

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

def run():
    subprocess.check_call(['./setup_ros.sh'])

    subprocess.check_call(['apt-get', 'update'])
    apt_opts = ['-y']
    env = os.environ.copy()
    env['DEBIAN_FRONTEND'] = 'noninteractive'
    subprocess.check_call(['apt-get'] + apt_opts + ['install'] + PACKAGES.split(),
                          env = env)
    subprocess.check_call(['apt-get'] + apt_opts + ['dist-upgrade'], env = env)
    subprocess.check_call(['apt-get', '-y', 'remove',
                           'network-manager-gnome', 'network-manager'])

    fcopy('/etc/hosts')
    ainsl('/etc/hosts', "127.0.1.1 %s" % socket.gethostname())
    fcopy('/etc/udev/rules.d/scarab.rules')
    fcopy('/etc/default/ifplugd')
    fcopy('/etc/wpa_supplicant/wpa_supplicant.conf')
    fcopy('/etc/olsrd.conf')
    subprocess.check_call(['./setup_interface.sh'])
    subprocess.check_call(['./setup_misc'])
    os.remove('/etc/udev/rules.d/70-persistent-net.rules')

if __name__ == "__main__":
    run()
