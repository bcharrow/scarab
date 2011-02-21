#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan m;
  m.header.stamp = msg->header.stamp;
  m.header.frame_id = msg->header.frame_id;
  m.angle_min = msg->angle_min;
  m.angle_max = msg->angle_max;
  m.angle_increment = 2.0*msg->angle_increment;
  m.time_increment = msg->time_increment;
  m.scan_time = msg->scan_time;
  m.range_min = msg->range_min;
  m.range_max = msg->range_max;
  m.intensities.resize(0);
  
  for (unsigned int i = 0; i < msg->ranges.size(); i++)
    if ((i % 2) == 0)
      m.ranges.push_back(msg->ranges[i]);

  pub.publish(m);
}

int main(int argc, char** argv)
{
  // Setup ROS
  ros::init(argc, argv, "laser_subsample");

  ros::NodeHandle n("~");

  ros::Subscriber sub = n.subscribe("scan", 1000, laser_callback);
  pub = n.advertise<sensor_msgs::LaserScan>("subscan", 100);

  ros::spin();

  return 0;
}


