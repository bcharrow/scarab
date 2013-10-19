#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>
#include <laser_simulator/ScanPair.h>

#include "matcher.hpp"
#include "Pose2d.hpp"

using namespace std;
using namespace mrsl;

// Print the low and high res costmaps
void print_map(const ScanMatcher &matcher) {
  static int id = 0;

  stringstream ss;
  const GridMap &map = matcher.map();
  for (int i = 0; i < map.height(); ++i) {
    for (int j = 0; j < map.width(); ++j) {
      ss << static_cast<int>(map.get(j, i)) << " ";
    }
  }
  ROS_DEBUG_NAMED("map", "%i %i %i %s",
                  id, map.width(), map.height(), ss.str().c_str());
  ++id;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "matcher_bag");
  if (argc != 2) {
    fprintf(stderr, "Usage: bag_matcher bagfile\n");
    return -1;
  }

  // ros::NodeHandle nh;
  MatchVisualizer vis;

  const char* fname = argv[1];
  rosbag::Bag bag;
  bag.open(fname, rosbag::bagmode::Read);

  vector<string> topics;
  topics.push_back("/data");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Time prev_time = ros::Time();

  ScanMatcher::Params p;
  p.grid_res = 0.02;
  p.sensor_sd = 0.02;
  ScanMatcher matcher(p);

  ScanMatcher::SearchWindow window;
  window.range_x = 1.4;
  window.range_y = 1.4;
  window.range_t = angles::from_degrees(45);
  window.inc_t = angles::from_degrees(0.5);

  tf::Transform true_transform;
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    if (!ros::ok()) {
      break;
    }
    laser_simulator::ScanPair::ConstPtr pair =
      m.instantiate<laser_simulator::ScanPair>();

    if (pair == NULL) {
      ROS_WARN("Non laser_simulator::ScanPair message on /data topic");
    }

    Eigen::Vector3d pose1_xyt(pair->pose1.position.x,
                              pair->pose1.position.y,
                              tf::getYaw(pair->pose1.orientation));
    Eigen::Vector3d pose2_xyt(pair->pose2.position.x,
                              pair->pose2.position.y,
                              tf::getYaw(pair->pose2.orientation));

    matcher.setScan(pose1_xyt, pair->scan1);

    // print_map(matcher);

    Gaussian3d delta = matcher.matchScan(pose2_xyt, pair->scan2, window);
    Pose2d initial_transform = Pose2d(pose2_xyt).ominus(Pose2d(pose1_xyt));
    Pose2d final = Pose2d(delta.mean()).oplus(initial_transform);
    vis.visualize(pair->scan1, pair->scan2, final.vector());
    tf::transformMsgToTF(pair->transform, true_transform);

    ROS_INFO_STREAM("Second:          " << pose2_xyt.transpose());
    ROS_INFO_STREAM("First:           " << pose1_xyt.transpose());
    ROS_INFO_STREAM("Initial:         " << initial_transform);
    ROS_INFO_STREAM("Delta:           " << delta.mean().transpose());
    ROS_INFO_STREAM("Final:           " << final);
    ROS_INFO("True:            %f %f %f",
             true_transform.getOrigin().x(), true_transform.getOrigin().y(),
             tf::getYaw(true_transform.getRotation()));
    // printf("%f % 0.5f % 0.5f % 0.5f % 0.5f % 0.5f % 0.5f\n",
    //        m.getTime().toSec(),
    //        final.x(), final.y(), final.t(),
    //        true_transform.getOrigin().x(), true_transform.getOrigin().y(),
    //        tf::getYaw(true_transform.getRotation()));
  }
}
