#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>

#include "matcher.hpp"

using std::string;

class LaserOdomNode {
public:
  LaserOdomNode()
    : pnh_("~"), matcher_(mrsl::ScanMatcher::Params::FromROS(pnh_)),
      have_pose_(false) {
    pnh_.param("odom_frame", odom_frame_, string("odom_laser"));
    pnh_.param("base_frame", base_frame_, string("base_link"));
    pnh_.param("debug", debug_, false);

    sscan_ = nh_.subscribe("scan", 5, &LaserOdomNode::laserCb, this);

    podom_ = nh_.advertise<nav_msgs::Odometry>("odom_laser", 5, false);
    if (debug_) {
      pscan_ = nh_.advertise<sensor_msgs::LaserScan>("scan_local", 1, false);
      pmap_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_local", 1, false);
    }

    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = base_frame_;
    
    matcher_.map().setFrameId(odom_frame_);
  }

  void laserCb(const sensor_msgs::LaserScan &scan) {
    // No odom estimate
    bool map_change = matcher_.addScan(Pose2d(0.0, 0.0, 0.0), scan);
    if (debug_) {
      sensor_msgs::LaserScan local_scan;
      local_scan = scan;
      local_scan.header.frame_id = base_frame_;
      pscan_.publish(local_scan);
      if (map_change) {
        pmap_.publish(matcher_.map().occGrid());
      }
    }

    odom_.header.stamp = scan.header.stamp;
    tf::poseTFToMsg(matcher_.pose().tf(), odom_.pose.pose);
    // Get velocity estimate
    if (!have_pose_) {
      last_pose_ = matcher_.pose();
      have_pose_ = true;
    } else {
      double dt = (scan.header.stamp - last_pose_time_).toSec();
      if (dt > 0.2) {
        Pose2d change = matcher_.pose().ominus(last_pose_);      
        odom_.twist.twist.linear.x = change.x() / dt;
        odom_.twist.twist.linear.y = change.y() / dt;
        odom_.twist.twist.angular.z = change.t() / dt;
        last_pose_time_ = scan.header.stamp;
        last_pose_ = matcher_.pose();
      }
    }

    podom_.publish(odom_);
    tf_.sendTransform(
      tf::StampedTransform(matcher_.pose().tf(), scan.header.stamp,
                           odom_frame_, base_frame_));
  }

private:
  ros::NodeHandle nh_, pnh_;
  std::string odom_frame_, base_frame_;
  mrsl::ScanMatcher matcher_;
  ros::Subscriber sscan_;
  ros::Publisher podom_, pmap_, pscan_;
  bool have_pose_;
  Pose2d last_pose_;
  ros::Time last_pose_time_;
  bool debug_;
  tf::TransformBroadcaster tf_;
  nav_msgs::Odometry odom_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_odom");

  LaserOdomNode lon;

  ros::spin();
}
