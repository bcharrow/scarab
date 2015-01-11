#! /usr/bin/env python

import roslib; roslib.load_manifest('hfn')
import rospy
import actionlib

from hfn.msg import *
from geometry_msgs.msg import PoseStamped

def callback(data):
    goal = MoveGoal()

    goal.target_poses.append(data)

    client.send_goal(goal)
    # client.wait_for_result(rospy.Duration.from_sec(5.0))

if __name__ == '__main__':
    rospy.init_node('move_client')
    client = actionlib.SimpleActionClient('move', MoveAction)
    client.wait_for_server()

    rospy.Subscriber("goal", PoseStamped, callback)
    rospy.spin()

