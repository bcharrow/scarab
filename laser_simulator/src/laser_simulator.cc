#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <odometry_aggregator/OdometryArray.h>
#include <LaserSimulator.h>
#include <tf/tf.h>

LaserSimulator sim;

void handle_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{
  sim.SetPose(msg->pose.pose);
  sim.SetFrameID(msg->child_frame_id);
}

double depth = 0;
bool map_set = false;

void handle_occupancy_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  sim.LoadOccupancyGrid(*msg, depth);
  map_set = true;
}

void handle_odom_array(const odometry_aggregator::OdometryArray::ConstPtr& msg)
{
  sim.UpdateOdometryArray(*msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_simulator");
  ros::NodeHandle n("~");

  ros::Subscriber odom_sub = n.subscribe("odom", 1, handle_odometry);
  ros::Subscriber grid_sub = n.subscribe("map", 1, handle_occupancy_grid);
  ros::Subscriber agg_sub = n.subscribe("odom_array", 1, handle_odom_array);

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param("depth", depth, 0.5);

  geometry_msgs::Pose offset;
  n.param("offset/x", offset.position.x, 0.0);
  n.param("offset/y", offset.position.y, 0.0);
  n.param("offset/z", offset.position.z, 0.0);
  double roll, pitch, yaw;
  n.param("offset/roll", roll, 0.0);
  n.param("offset/pitch", pitch, 0.0);
  n.param("offset/yaw", yaw, 0.0);

  btQuaternion quat;
  quat.setEuler(yaw, pitch, roll);
  quat.normalize();

  offset.orientation.x = quat.x();
  offset.orientation.y = quat.y();
  offset.orientation.z = quat.z();
  offset.orientation.w = quat.w();

  sim.SetLaserOffset(offset);

  if (sim.LoadLaserModel(n) != 0)
    {
      ROS_ERROR("%s: failed to load laser model", 
                ros::this_node::getName().c_str());
      return -1;
    }

  ros::Rate idle(1);
  while (n.ok())
    {
      if (map_set)
        break;     
      else
        ROS_DEBUG("%s: waiting for map", 
                  ros::this_node::getName().c_str());

      ros::spinOnce();
      idle.sleep();
    }

  sensor_msgs::LaserScan msg;
  
  std::string frame_id;
  n.param("frame_id", frame_id, std::string("laser"));
  msg.header.frame_id = frame_id;
  msg.angle_min = sim.GetMinimumAngle();
  msg.angle_max = sim.GetMaximumAngle();
  msg.angle_increment = sim.GetAngleIncrement();
  msg.time_increment = sim.GetTimeIncrement();
  msg.scan_time = sim.GetScanTime();
  msg.range_min = sim.GetMinimumRange();
  msg.range_max = sim.GetMaximumRange();
  msg.ranges.resize((msg.angle_max - msg.angle_min)/msg.angle_increment);
  msg.intensities.resize(0);

  ros::Rate r(1.0/msg.scan_time);
  while (n.ok())
    {
      sim.GetScan(msg.ranges);

      msg.header.stamp = ros::Time::now();
      pub.publish(msg);
     
      ros::spinOnce();
      r.sleep();
    }

  return 0;
}
