#include <ros/ros.h>
#include <ipc_bridge/ipc_bridge.h>

#include <odometry_aggregator/OdometryArray.h>
#include <ipc_bridge/msgs/odometry_aggregator_OdometryArray.h>

#define NAMESPACE odometry_aggregator
#define NAME OdometryArray

ros::Publisher pub;
NAMESPACE::NAME out_msg;

void callback(const ipc_bridge::NAMESPACE::NAME &msg)
{ 
  out_msg.header.seq = msg.header.seq;
  out_msg.header.stamp = ros::Time(msg.header.stamp);
  out_msg.header.frame_id = std::string(msg.header.frame_id);

  // TODO: Implement
  ROS_WARN("%s: not fully implemented", 
           ros::this_node::getName().c_str());

  pub.publish(out_msg);
}

#include "subscriber.h"
