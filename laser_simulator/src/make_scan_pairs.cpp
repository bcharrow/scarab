#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <LaserSimulator.h>
#include <tf/tf.h>

#include <laser_simulator/ScanPair.h>

#include <rosbag/bag.h>

#include <ctime>
#include <boost/foreach.hpp>

using namespace std;

LaserSimulator sim;

double depth = 0;
bool map_set = false;

void handle_occupancy_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  sim.LoadOccupancyGrid(*msg, depth);
  map_set = true;
}

void straight(tf::Pose start_pose, double step_size, int nsteps,
              laser_simulator::ScanPair *pair, rosbag::Bag *bag) {
  tf::Transform tform(tf::createQuaternionFromYaw(0.0),
                      tf::Vector3(step_size, 0, 0));
  tf::Pose stop_pose;
  for (int i = 1; i <= nsteps; ++i) {
    stop_pose.mult(start_pose, tform);

    tf::poseTFToMsg(start_pose, pair->pose1);
    sim.SetPose(pair->pose1);
    sim.GetScan(pair->scan1.ranges);

    tf::poseTFToMsg(stop_pose, pair->pose2);
    sim.SetPose(pair->pose2);
    sim.GetScan(pair->scan2.ranges);

    tf::transformTFToMsg(tform, pair->transform);

    bag->write("/data", ros::Time::now(), *pair);

    start_pose = stop_pose;
  }
}

void sim_pairs(rosbag::Bag *bag) {
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

  tf::Quaternion quat;
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
      ROS_BREAK();
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

  laser_simulator::ScanPair scan_pair;
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
  straight(start_pose, 0.1, 20, &scan_pair, bag);
}

void from_bag(const string &input_fname, rosbag::Bag *out_bag) {
  ros::NodeHandle n("~");
  string laser_topic, pose_topic;
  double min_sep;
  n.param("laser_topic", laser_topic, string("/laser"));
  n.param("pose_topic", pose_topic, string("/amcl_pose"));
  n.param("min_sep", min_sep, 0.25);
  ROS_INFO("Reading %s and %s from %s to generate scan pairs",
           laser_topic.c_str(), pose_topic.c_str(), input_fname.c_str());
  rosbag::Bag input;
  input.open(input_fname.c_str(), rosbag::bagmode::Read);

  // Build TF tree
  tf::Transformer transformer(true, ros::Duration(10000));
  vector<string> topics;
  topics.push_back(pose_topic);
  rosbag::View pose_view(input, rosbag::TopicQuery(topics));
  BOOST_FOREACH(rosbag::MessageInstance const m, pose_view) {
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose3d;
    pose3d = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    if (pose3d == NULL) {
      ROS_ERROR("Non PoseWithCovarianceStamped on %s", pose_topic.c_str());
      ROS_BREAK();
    }
    tf::StampedTransform stamped;
    stamped.stamp_ = pose3d->header.stamp;
    stamped.child_frame_id_ = "/bot";
    stamped.frame_id_ = "/map";
    tf::Transform transform;
    tf::poseMsgToTF(pose3d->pose.pose, transform);
    stamped.setData(transform);

    stringstream ss;
    ss << pose3d->pose.pose;
    // ROS_INFO_STREAM("" << stamped.stamp_ << " " <<
    //                 stamped.child_frame_id_ << " " <<
    //                 stamped.frame_id_.c_str() << " " <<
    //                 ss.str());

    transformer.setTransform(stamped);
  }

  topics[0] = laser_topic;
  rosbag::View laser_view(input, rosbag::TopicQuery(topics));
  sensor_msgs::LaserScan::ConstPtr laser_first, laser_second, laser;

  laser_simulator::ScanPair pair;
  BOOST_FOREACH(rosbag::MessageInstance &m, laser_view) {
    laser = m.instantiate<sensor_msgs::LaserScan>();
    if (laser == NULL) {
      ROS_ERROR("Non laser scan on %s", laser_topic.c_str());
      ROS_BREAK();
    }

    if (laser_first == NULL) {
      laser_first = laser;
      continue;
    } else {
      laser_second = laser;
    }

    if ((laser_second->header.stamp - laser_first->header.stamp).toSec() <
        min_sep) {
      continue;
    }

    pair.scan1 = *laser_first;
    pair.scan2 = *laser_second;

    tf::StampedTransform truth;
    try {
      // ROS_INFO_STREAM("" << laser_first->header.stamp << " " <<
      //                 laser_second->header.stamp);
      transformer.lookupTransform("/bot", laser_first->header.stamp,
                                  "/bot", laser_second->header.stamp,
                                  "/map", truth);
    } catch (exception &e) {
      ROS_WARN("%s", e.what());
      laser_first = laser_second;
      continue;
    }
    tf::transformTFToMsg(truth, pair.transform);
    out_bag->write("/data", ros::Time::now(), pair);

    laser_first = laser_second;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "make_scan_pairs");
  ros::NodeHandle n("~");
  string dir;
  n.param("dir", dir, string(""));
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

  if (argc == 1) {
    ROS_INFO("Making scan pairs from simulation");
    sim_pairs(&bag);
  } else {
    string input_bag = argv[1];
    from_bag(input_bag, &bag);
  }
  return 0;
}
