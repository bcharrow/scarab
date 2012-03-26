#!/usr/bin/env python
#
# A node that listens for PoseWithCovarianceStamped and publishes PoseStamped
import roslib; roslib.load_manifest('scarab_rviz')
import rospy

import threading

import tf

import sensor_msgs.msg
import geometry_msgs.msg

class Stripper(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._pub = rospy.Publisher("pose_stamped", geometry_msgs.msg.PoseStamped)
        self._pose_sub = rospy.Subscriber("amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped,
                                          self._pose_callback)
        self._xyz = None
        self._orient = None
        self._br = tf.TransformBroadcaster()
    def _pose_callback(self, pose_msg):
        with self._lock:
            pose = pose_msg.pose.pose
            msg = geometry_msgs.msg.PoseStamped(header = pose_msg.header,
                                                pose = pose)
            msg.header.frame_id = '/map'
            self._pub.publish(msg)
            p = pose.position
            o = pose.orientation
            self._xyz = (p.x, p.y, p.z)
            self._orient = (o.x, o.y, o.z, o.w)

    def broadcast(self):
        with self._lock:
            if self._xyz is not None:
                self._br.sendTransform(self._xyz, self._orient,
                                       rospy.Time.now(),
                                       '%swlaser' % rospy.get_namespace(), "/map")

def main():
    rospy.init_node('range_pose_aggregator')
    strip = Stripper()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        strip.broadcast()
        r.sleep()

if __name__ == "__main__":
    main()
