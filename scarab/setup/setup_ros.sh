#!/bin/bash
set -o errexit

echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list

wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
