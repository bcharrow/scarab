#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

using namespace std;

geometry_msgs::Twist twist;
ros::Publisher pub;
ros::NodeHandle *node;
double max_v = 0.15;
double max_w = 1.0;

int linear_axis;
int angular_axis;
bool flip_angular;

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  double v_analog = msg->axes[linear_axis]*max_v;
//  double v_digital = msg->axes[5]*max_v;
  double w_analog = msg->axes[angular_axis]*max_w;
//  double w_digital = msg->axes[4]*max_w;

//  double v = (fabs(v_analog) > fabs(v_digital))?v_analog:v_digital;
//  double w = (fabs(w_analog) > fabs(w_digital))?w_analog:w_digital;
  double v = v_analog;
  double w = flip_angular ? -w_analog: w_analog;
  
  twist.linear.x  = v;
  twist.angular.z = w;
  pub.publish(twist);
}

bool spin()
{
  ros::Rate r(20);
  while(node->ok()) 
  {
    ros::spinOnce();
    r.sleep();
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_to_twist");
  ros::NodeHandle n("~");
  node = &n;

  std::string input_topic;
  std::string output_topic;
  node->param("vmax", max_v, 0.15);
  node->param("wmax", max_w, 1.0);
  node->param("linear_axis", linear_axis, 1);
  node->param("angular_axis", angular_axis, 0);
  node->param("flip", flip_angular, false);
  node->param("input_topic", input_topic, std::string("joy"));
  node->param("output_topic", output_topic, std::string("twist"));
  
  ros::Subscriber sub1 = n.subscribe(input_topic, 10, joy_callback);
  pub = n.advertise<geometry_msgs::Twist>(output_topic, 10);

  spin();

  return 0;
}


