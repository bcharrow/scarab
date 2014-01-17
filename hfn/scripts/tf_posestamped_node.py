#!/usr/bin/env python

import roslib; roslib.load_manifest('hfn')
import rospy
import tf

from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('pose_stamped')

    base_frame = rospy.get_param("~base_frame_id", "base")
    map_frame = rospy.get_param("~map_frame_id", "map")
    pose_pub = rospy.Publisher("pose", PoseStamped)

    tf_sub = tf.TransformListener()

    rate = rospy.Rate(10.0)
    last_pub = rospy.Time.now()
    while not rospy.is_shutdown():
        dur = last_pub - rospy.Time.now()
        if dur > rospy.Duration(5.0):
            rospy.logwarn("tf_posestamped_node: Haven't published pose in %f seconds",
                          dur.to_sec())
        try:
            (trans,rot) = tf_sub.lookupTransform(map_frame, base_frame,
                                                 rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logwarn("%s" % e)
            rospy.sleep(1.0)
            continue

        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]

        pose_pub.publish(pose)
        last_pub = rospy.Time.now()

        rate.sleep()

if __name__ == "__main__":
    main()
