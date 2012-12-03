#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <odometry_aggregator/OdometryArray.h>
#include <LaserSimulator.h>
#include <tf/tf.h>

#include <laser_simulator/ScanPair.h>

#include <rosbag/bag.h>

#include <ctime>

using namespace std;

LaserSimulator sim;
laser_simulator::ScanPair scan_pair;

void handle_odometry(const nav_msgs::Odometry::ConstPtr& odom) {
  sim.SetPose(odom->pose.pose);
  sim.SetFrameID(odom->child_frame_id);
}

double depth = 0;
bool map_set = false;

void handle_occupancy_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  sim.LoadOccupancyGrid(*msg, depth);
  map_set = true;
}

void straight(tf::Pose start_pose, double step_size, int nsteps, rosbag::Bag *bag) {
  tf::Transform tform(tf::createQuaternionFromYaw(0.0),
                      tf::Vector3(step_size, 0, 0));
  tf::Pose stop_pose;
  for (int i = 1; i <= nsteps; ++i) {
    stop_pose.mult(start_pose, tform);

    tf::poseTFToMsg(start_pose, scan_pair.pose1);
    sim.SetPose(scan_pair.pose1);
    sim.GetScan(scan_pair.scan1.ranges);

    tf::poseTFToMsg(stop_pose, scan_pair.pose2);
    sim.SetPose(scan_pair.pose2);
    sim.GetScan(scan_pair.scan2.ranges);
    
    tf::transformTFToMsg(tform, scan_pair.transform);
    
    bag->write("/data", ros::Time::now(), scan_pair);
    
    start_pose = stop_pose;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_simulator");
  ros::NodeHandle n("~");

  ros::Subscriber grid_sub = n.subscribe("map", 1, handle_occupancy_grid);

  n.param("depth", depth, 0.5);

  geometry_msgs::Pose offset;
  n.param("offset/x", offset.position.x, 0.0);
  n.param("offset/y", offset.position.y, 0.0);
  n.param("offset/z", offset.position.z, 0.0);
  double roll, pitch, yaw;
  n.param("offset/roll", roll, 0.0);
  n.param("offset/pitch", pitch, 0.0);
  n.param("offset/yaw", yaw, 0.0);

  string dir;
  n.param("dir", dir, string(""));
  
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

  char date[100];
  time_t t;
  struct tm *tm;
  t = time(NULL);
  tm = localtime(&t);
  strftime(date, sizeof(date) / sizeof(date[0]), "%Y-%m-%d-%H-%M-%S", tm);
  
  rosbag::Bag bag;
  string path = string(date) + ".bag";
  if (dir.size() != 0) {
    path = dir + "/" + path;
  }

  ROS_INFO("Saving scan pairs to: %s", path.c_str());
  bag.open(path.c_str(), rosbag::bagmode::Write);
  std::string frame_id;
  n.param("frame_id", frame_id, std::string("laser"));
  scan_pair.scan1.header.frame_id = frame_id;
  scan_pair.scan1.angle_min = sim.GetMinimumAngle();
  scan_pair.scan1.angle_max = sim.GetMaximumAngle();
  scan_pair.scan1.angle_increment = sim.GetAngleIncrement();
  scan_pair.scan1.range_min = sim.GetMinimumRange();
  scan_pair.scan1.range_max = sim.GetMaximumRange();
  int nranges = (scan_pair.scan1.angle_max - scan_pair.scan1.angle_min) /
    scan_pair.scan1.angle_increment + 1;
  scan_pair.scan1.ranges.resize(nranges);
  scan_pair.scan1.intensities.resize(0);
  scan_pair.scan1.scan_time = 0.001;
  scan_pair.scan1.time_increment = scan_pair.scan1.scan_time / sim.GetScanCount();
  scan_pair.scan2 = scan_pair.scan1;

  tf::Pose start_pose(tf::createQuaternionFromYaw(0.0), tf::Vector3(-7.4, 4.4, 0.0));
  straight(start_pose, 0.1, 20, &bag);

  return 0;
}
