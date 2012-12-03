#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <odometry_aggregator/OdometryArray.h>
#include <LaserSimulator.h>
#include <tf/tf.h>

LaserSimulator sim;
ros::Publisher pub;
sensor_msgs::LaserScan msg;
boost::mutex sim_mutex;

void handle_odometry(const nav_msgs::Odometry::ConstPtr& odom)
{
  boost::mutex::scoped_lock(sim_mutex);
  msg.header.stamp = odom->header.stamp;
  msg.header.stamp.nsec += 1000;
  sim.SetPose(odom->pose.pose);
  sim.SetFrameID(odom->child_frame_id);
}

double depth = 0;
bool map_set = false;

void handle_occupancy_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  boost::mutex::scoped_lock(sim_mutex);
  sim.LoadOccupancyGrid(*msg, depth);
  map_set = true;
}

void handle_odom_array(const odometry_aggregator::OdometryArray::ConstPtr& msg)
{
  boost::mutex::scoped_lock(sim_mutex);
  sim.UpdateOdometryArray(*msg);
}

void publish(const ros::TimerEvent&) {
  boost::mutex::scoped_lock(sim_mutex);
  sim.GetScan(msg.ranges);
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_simulator");
  ros::NodeHandle n("~");

  ros::Subscriber odom_sub = n.subscribe("odom", 1, handle_odometry);
  ros::Subscriber grid_sub = n.subscribe("map", 1, handle_occupancy_grid);
  ros::Subscriber agg_sub = n.subscribe("odom_array", 1, handle_odom_array);

  pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

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

  
  std::string frame_id;
  n.param("frame_id", frame_id, std::string("laser"));
  msg.header.frame_id = frame_id;
  msg.angle_min = sim.GetMinimumAngle();
  msg.angle_max = sim.GetMaximumAngle();
  msg.angle_increment = sim.GetAngleIncrement();
  msg.range_min = sim.GetMinimumRange();
  msg.range_max = sim.GetMaximumRange();
  msg.ranges.resize((msg.angle_max - msg.angle_min)/msg.angle_increment + 1);
  msg.intensities.resize(0);
  msg.scan_time = 0.001;
  msg.time_increment = msg.scan_time / sim.GetScanCount();

  ros::Timer timer = n.createTimer(ros::Duration(sim.GetScanTime()), &publish);

  ros::spin();

  return 0;
}
