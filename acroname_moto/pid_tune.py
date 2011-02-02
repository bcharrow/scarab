#!/usr/bin/env python

import sys
import roslib; roslib.load_manifest('acroname_moto')

import curses

import rospy
from DifferentialDriveMsgs.msg import PIDParam
from geometry_msgs.msg import Twist, Vector3


class RosConnection():
    p = 0
    i = 0
    d = 0
    period = 0
    fresh = False
    def __init__(self, prefix):
        self.vel_pub = rospy.Publisher(prefix+'cmd_vel', Twist)
        self.param_pub = rospy.Publisher(prefix+'tuning_input', PIDParam)
        rospy.init_node('pid_tune', anonymous=True)
        rospy.Subscriber(prefix+'tuning_output', PIDParam, self.HandlePIDParam)

    def SendParam(self, p, i, d, period):
        self.param_pub.publish(PIDParam(p, i, d, period))

    def SendVel(self, v, w):
        self.vel_pub.publish(Twist(linear=Vector3(v, 0, 0),
                                   angular=Vector3(0, 0, w)))

    def HandlePIDParam(self, data):
        self.p = data.p
        self.i = data.i
        self.d = data.d
        self.period = data.period
        self.fresh = True

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        prefix = sys.argv[1]
    else:
        prefix = '/'

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(1)

    conn = RosConnection(prefix)
    done = False
    v = 0
    w = 0
    p = 0
    i = 0
    d = 0
    period = 0
    stdscr.addstr(0, 0, 'Control set-point velocity with Up/Down/Left/Right')
    stdscr.addstr(1, 0, 'Zero command with /, Send cmd with <space>, Quit with q')
    stdscr.nodelay(1)
    while not done:
        stdscr.addstr(2, 0, 'V: %2.2f, W: %2.2f      ' % (v, w))
        if conn.fresh:
            p = conn.p
            i = conn.i
            d = conn.d
            period = conn.period
            conn.fresh = False

        stdscr.addstr(3, 0, 'p,i,d: %2.2f, %2.2f, %2.2f        ' % (p, i, d))
            
        c = stdscr.getch()
        if c == ord('q'):
            done = True
            conn.SendVel(0, 0)            
        elif c == ord('/'):
            conn.SendVel(0, 0)
        elif c == ord(' '):
            conn.SendParam(p, i, d, period)
            conn.SendVel(v, w)
        elif c == curses.KEY_UP:
            v += 0.05
        elif c == curses.KEY_DOWN:
            v -= 0.05
        elif c == curses.KEY_RIGHT:
            w -= 0.1
        elif c == curses.KEY_LEFT:
            w += 0.1
        elif c == ord('P'):
            p += 0.1
        elif c == ord('p'):
            p -= 0.1
        elif c == ord('I'):
            i += 0.1
        elif c == ord('i'):
            i -= 0.1
        elif c == ord('D'):
            d += 0.1
        elif c == ord('d'):
            d -= 0.1

    curses.nocbreak(); stdscr.keypad(0); curses.echo()
    curses.endwin()
