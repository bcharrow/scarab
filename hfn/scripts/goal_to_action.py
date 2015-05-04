#! /usr/bin/env python

import roslib; roslib.load_manifest('hfn')
import rospy
import actionlib

from scarab_msgs.msg import *
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import math

# set_height = None
# yaw_adjust = 0.0

def callback(data):
    goal = MoveGoal()
    rospy.loginfo("Got goal to %s" % data)

    if set_height is not None and math.fabs(data.pose.position.z) < 1e-3:
        data.pose.position.z = set_height

    rot = tft.numpy.array([data.pose.orientation.x,
                           data.pose.orientation.y,
                           data.pose.orientation.z,
                           data.pose.orientation.w])
    adjust = tft.quaternion_from_euler(0.0, 0.0, yaw_adjust)
    orientation = tft.quaternion_multiply(rot, adjust)
    data.pose.orientation.x = orientation[0]
    data.pose.orientation.y = orientation[1]
    data.pose.orientation.z = orientation[2]
    data.pose.orientation.w = orientation[3]

    goal.target_poses.append(data)

    client.send_goal(goal)
    # client.wait_for_result(rospy.Duration.from_sec(5.0))

if __name__ == '__main__':
    global set_height
    global yaw_adjust

    rospy.init_node('move_client')
    client = actionlib.SimpleActionClient('move', MoveAction)
    rospy.loginfo("Waiting for server")
    client.wait_for_server()
    rospy.loginfo("Connected!")

    set_height = rospy.get_param("~z", None)
    yaw_adjust = rospy.get_param("~yaw_adjust", 0.0)

    rospy.Subscriber("goal", PoseStamped, callback)
    rospy.spin()

