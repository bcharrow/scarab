#!/usr/bin/env python

import sys
import os
import time
import math

import roslib; roslib.load_manifest('roboclaw')
import rospy
import tf.transformations

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from collections import namedtuple

class Trial(object):
    def __init__(self, robot):
        robot_odom = robot + '/motor/odom'
        motor_topic = robot + '/cmd_vel'
        print "Waiting for %s" % (robot_odom)
        self.handle_odom_pose(rospy.wait_for_message(robot_odom, Odometry))
        rospy.Subscriber(robot_odom, Odometry, self.handle_odom_pose)
        self.motor_pub = rospy.Publisher(motor_topic, Twist, latch = True)

        self.data = []

    def _command(self, twist, duration, distance, angle):
        # Record position
        odom_start = self.odom_pose
        print odom_start

        # Move
        self.motor_pub.publish(twist)
        if distance == 0:
            time.sleep(1.0)
            while abs(get_yaw(odom_start) - get_yaw(self.odom_pose)) > 0.05:
                time.sleep(0.01)
        else:
            while (l2(odom_start, self.odom_pose) < distance - 0.02):
                time.sleep(0.01)

        # Stop
        t = Twist()
        t.linear.x = 0
        t.angular.z = 0
        self.motor_pub.publish(t)

        # Wait for bot to come to a stop
        rospy.sleep(1.0)

        # Record final location
        odom_end = self.odom_pose
        print odom_end
        self.data.append((odom_start, odom_end, distance, angle))

    def drive(self, distance):
        # distance is in meters
        print "Starting a translation trial...",
        sys.stdout.flush()
        t = Twist()
        t.linear.x = 0.2
        duration  = distance / math.fabs(t.linear.x)

        self._command(t, duration, distance, 0)
        print "done"

    def turn(self, angle):
        # angle is in radians
        print "Starting a rotation trial...",
        sys.stdout.flush()
        t = Twist()
        t.angular.z = -0.6
        duration  = angle / math.fabs(t.angular.z)

        self._command(t, duration, 0, angle)
        print "done"


    def handle_odom_pose(self, msg):
        self.odom_pose = msg.pose.pose

def l2(pose1, pose2):
    return math.sqrt((pose1.position.x - pose2.position.x)**2 +
                     (pose1.position.y - pose2.position.y)**2)

def get_yaw(pose):
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return tf.transformations.euler_from_quaternion(quat)[2]

def angle_err(pose1, pose2):
    return math.fabs(getYaw(pose1.orientation) - getYaw(pose2.orientation))

def pose_string(pose, fmt = "0.2f"):
    debug = "(x, y, theta): ({pos.x:{fmt:s}}, {pos.y:{fmt:s}}, {yaw:{fmt:s}})"
    return  debug.format(pos = pose.position, yaw = get_yaw(pose), fmt=fmt)

def run(name, trials = 1, distance = 0.75):
    rospy.init_node('calibrate')
    t = Trial(name)
    for i in range(trials):
        if not rospy.is_shutdown():
            # t.drive(distance = 1.0)
            # rospy.sleep(2.0)
            t.turn(math.pi * 2)

    odom_err_ttl = 0
    for odom_start, odom_end, distance, angle in t.data:
        # print "Odom Start " + pose_string(odom_start)
        # print "Odom End   " + pose_string(odom_end)

        print odom_start, odom_end
        odom_distance = l2(odom_start, odom_end)

        odom_err = math.fabs(odom_distance - distance)
        odom_err_ttl += odom_err
        print "Commanded Dist: %0.2f Odom Distance: %0.2f Error: %0.2f" % (
            distance, odom_distance, odom_err)
        print "Start angle: %f End Angle: %f Diff: %f" % (
            get_yaw(odom_start), get_yaw(odom_end), get_yaw(odom_end) - get_yaw(odom_start))

    print "Odometry Dist. Total Error:  %0.2f" % odom_err_ttl

if __name__ == "__main__":
    name = rospy.resolve_name(os.getenv('ROS_NAMESPACE'))
    run(name)
