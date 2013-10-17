#!/usr/bin/env python
#
# A node that listens for PoseWithCovarianceStamped and publishes PoseStamped
import roslib; roslib.load_manifest('scarab')
import rospy

import geometry_msgs.msg

class Stripper(object):
    def __init__(self):
        self._pub = rospy.Publisher("pose_stamped",
                                    geometry_msgs.msg.PoseStamped)
        self._pose_sub = \
            rospy.Subscriber("amcl_pose",
                             geometry_msgs.msg.PoseWithCovarianceStamped,
                             self._pose_callback)

    def _pose_callback(self, pose_msg):
        pose = pose_msg.pose.pose
        msg = geometry_msgs.msg.PoseStamped(header = pose_msg.header,
                                            pose = pose)
        msg.header.frame_id = pose_msg.header.frame_id
        if (msg.header.frame_id[0] != '/'):
          msg.header.frame_id = "/" + msg.header.frame_id
        self._pub.publish(msg)

def main():
    rospy.init_node('posestamped_node')
    strip = Stripper()
    rospy.spin()

if __name__ == "__main__":
    main()
