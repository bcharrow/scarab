#include <ros/ros.h>
#include <ipc_bridge/ipc_bridge.h>

#include <odometry_aggregator/OdometryArray.h>
#include <ipc_bridge/msgs/odometry_aggregator_OdometryArray.h>

#define NAMESPACE odometry_aggregator
#define NAME OdometryArray

ipc_bridge::Publisher<ipc_bridge::NAMESPACE::NAME> *p;
ipc_bridge::NAMESPACE::NAME out_msg;

unsigned int frame_id_prior_size = 0;

void callback(const NAMESPACE::NAME::ConstPtr &msg)
{   
  out_msg.header.seq = msg->header.seq;
  out_msg.header.stamp = msg->header.stamp.toSec();

  if (strlen(msg->header.frame_id.c_str()) != frame_id_prior_size)
    {
      if (out_msg.header.frame_id != 0)
        delete[] out_msg.header.frame_id;

      out_msg.header.frame_id = 
        new char[strlen(msg->header.frame_id.c_str()) + 1];
      strcpy(out_msg.header.frame_id, msg->header.frame_id.c_str());
      frame_id_prior_size = strlen(msg->header.frame_id.c_str());
    }

  out_msg.array_length = msg->array.size();
  out_msg.array = new nav_msgs_Odometry[out_msg.array_length];
  
  for (unsigned int i = 0; i < out_msg.array_length; i++)
    {
      out_msg.array[i].header.seq = msg->array[i].header.seq;
      out_msg.array[i].header.stamp = msg->array[i].header.stamp.toSec();              
      out_msg.array[i].header.frame_id = 
        new char[strlen(msg->array[i].header.frame_id.c_str()) + 1];
      strcpy(out_msg.array[i].header.frame_id, msg->array[i].header.frame_id.c_str());
            
      out_msg.array[i].child_frame_id = 
        new char[strlen(msg->array[i].child_frame_id.c_str()) + 1];
      strcpy(out_msg.array[i].child_frame_id, msg->array[i].child_frame_id.c_str());
           
      out_msg.array[i].pose.pose.position.x = msg->array[i].pose.pose.position.x;
      out_msg.array[i].pose.pose.position.y = msg->array[i].pose.pose.position.y;
      out_msg.array[i].pose.pose.position.z = msg->array[i].pose.pose.position.z;
      
      out_msg.array[i].pose.pose.orientation.x = msg->array[i].pose.pose.orientation.x;
      out_msg.array[i].pose.pose.orientation.y = msg->array[i].pose.pose.orientation.y;
      out_msg.array[i].pose.pose.orientation.z = msg->array[i].pose.pose.orientation.z;
      out_msg.array[i].pose.pose.orientation.w = msg->array[i].pose.pose.orientation.w;

      std::copy(msg->array[i].pose.covariance.begin(),
                msg->array[i].pose.covariance.end(),
                out_msg.array[i].pose.covariance);

      out_msg.array[i].twist.twist.linear.x = msg->array[i].twist.twist.linear.x;
      out_msg.array[i].twist.twist.linear.y = msg->array[i].twist.twist.linear.y;
      out_msg.array[i].twist.twist.linear.z = msg->array[i].twist.twist.linear.z;
      
      out_msg.array[i].twist.twist.angular.x = msg->array[i].twist.twist.angular.x;
      out_msg.array[i].twist.twist.angular.y = msg->array[i].twist.twist.angular.y;
      out_msg.array[i].twist.twist.angular.z = msg->array[i].twist.twist.angular.z;

      std::copy(msg->array[i].twist.covariance.begin(),
                msg->array[i].twist.covariance.end(),
                out_msg.array[i].twist.covariance);
    }
  
  p->Publish(out_msg);

  if (out_msg.array != 0)
    delete[] out_msg.array;

  out_msg.array = 0;
}
#include "publisher.h"
