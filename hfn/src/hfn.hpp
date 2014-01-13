#ifndef HFN_HPP
#define HFN_HPP

#include <boost/scoped_ptr.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>

#include <hfn/MoveAction.h>

#include "player_map/rosmap.hpp"

namespace scarab {

class HumanFriendlyNav {
public:
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef K::Vector_2 Vector_2;
  typedef CGAL::Polygon_2<K> Polygon_2;

  struct Params {
    double axle_width;
    double robot_radius;
    double safety_margin;
    double social_margin;
    double waypoint_thresh;
    double alpha_thresh;
    double tau_1;
    double tau_2;
    double tau_r;
    double w_max;
    double v_opt;

    double freq;
    std::string map_frame;
    std::string base_frame;
  };

  HumanFriendlyNav(Params p);
  static HumanFriendlyNav* ROSInit(ros::NodeHandle& nh);

  void setGoal(const geometry_msgs::PoseStamped &input);
  // Note: May be different than last call to setGoal() due to goal projection
  void getGoal(geometry_msgs::PoseStamped *input) {
    input->header.frame_id = params_.base_frame;
    input->pose = goal_;
  }

  void setLaserScan(const sensor_msgs::LaserScan &input);
  void setPose(const geometry_msgs::PoseStamped &input);
  void setOdom(const nav_msgs::Odometry &input);
  void getCommandVel(geometry_msgs::Twist *cmd_vel);

  const sensor_msgs::LaserScan& inflatedScan() {
    return free_distance_;
  }

  const Polygon_2& inflatedPolygon() {
    return polygon_;
  }

  const Params &params() { return params_; }

private:
  void desiredOrientation(double &alpha_des, double &distance_des);
  void orientationToTwist(double alpha, double distance, geometry_msgs::Twist &twist);
  double desiredVelocity(double distance, double alpha);

  void freeDistance(const sensor_msgs::LaserScan &input);
  void addObstacle(std::vector<float>::iterator &it, float scan_dist);
  float calcReducedRange(float d, float r, float theta);
  float obstacleRadius();
  void twistToWheelVel(const geometry_msgs::Twist &twist, double &left, double &right);
  void wheelVelToTwist(double left, double right, geometry_msgs::Twist *twist);

  Params params_;
  sensor_msgs::LaserScan free_distance_;
  geometry_msgs::Pose pose_, goal_;
  geometry_msgs::Twist current_twist_, goal_twist_;
  Polygon_2 polygon_;  // Polygon of free_distance_
};

class HFNWrapper {
public:
  struct Params {
    double max_occ_dist;     // maximum margin for distance transform
    double lethal_occ_dist;  // distance at which robot is in collision
    double cost_occ_prob;    // factor to convert occupancy prob to cost
    double cost_occ_dist;    // factor to convert occ_dist to cost
    double goal_tol;         // distance when goal is considered reached
    double path_margin;      // max margin for path planning
    double waypoint_spacing; // max distance between waypoints
    double los_margin;       // margin for line of sight checks
    double timeout;          // time at which an action is aborted
    double stuck_distance;   // radius of neighborhood for when a robot is stuck
    double stuck_angle;      // Not stuck if moved angle is larger than this
    double stuck_timeout;    // time over which a robot must remain in radius
    int free_threshold;      // max occupancy grid value for free space
    int occupied_threshold;  // min occupancy grid value for occupied space
    bool allow_unknown_path; // allow paths through unknown space
    bool allow_unknown_los;  // allow line of sight through unknown space
    std::string map_frame;
    std::string name_space;
  };

  enum Status {
    FINISHED,   // Successfully reached goal
    TIMEOUT,    // Took too long to reach goal
    STUCK,      // Robot has not moved in a while
    NOTREADY,   // Don't have all the data we need to move, check back soon!
    UNREACHABLE // Goal is no longer reachable (e.g., due to map change)
  };

  HFNWrapper(const Params &params, HumanFriendlyNav *hfn);
  ~HFNWrapper();

  static HFNWrapper* ROSInit(ros::NodeHandle& nh);

  void onPose(const geometry_msgs::PoseStamped &input);
  void onMap(const nav_msgs::OccupancyGrid &input);
  void onLaserScan(const sensor_msgs::LaserScan &scan);
  void onOdom(const nav_msgs::Odometry &odom);
  void stop();
  void setGoal(const std::vector<geometry_msgs::PoseStamped> &p);
  void registerStatusCallback(const boost::function<void(Status)> &callback);

private:
  void timeout(const ros::TimerEvent &event);

  // Return true if we can follow a waypoint, false otherwise
  bool updateWaypoint();
  void pubWaypoints();
  void pubPolygon(const HumanFriendlyNav::Polygon_2 &polygon);
  bool initialized() {
    return flags_.have_pose && flags_.have_odom && flags_.have_map &&
      flags_.have_laser;
  }
  // Get string describing which things have not been initialized yet
  std::string uninitializedString();

  ros::NodeHandle nh_;
  ros::Publisher path_pub_, vis_pub_, vel_pub_, inflated_pub_, costmap_pub_;
  ros::Subscriber pose_sub_, map_sub_, odom_sub_, laser_sub_;

  boost::function<void(Status)> callback_;
  bool active_; // True if we're navigating to a goal
  geometry_msgs::PoseStamped pose_;
  std::vector<geometry_msgs::PoseStamped> goals_;
  std::list<geometry_msgs::PoseStamped> pose_history_;
  scarab::Path waypoints_;
  boost::scoped_ptr<scarab::OccupancyMap> map_;
  Params params_;
  HumanFriendlyNav *hfn_;
  ros::Timer timeout_timer_;
  struct {
    bool have_pose, have_odom, have_map, have_laser;
  } flags_;
};

class MoveServer {
public:
  MoveServer(const std::string &server_name, HFNWrapper *wrapper);
  void executeCB(const hfn::MoveGoalConstPtr &goal);
  void start() {
    as_.start();
  }

  void stop() {
    wrapper_->stop();
    as_.shutdown();
  }

  void goalCallback();
  void preemptCallback();
  void hfnCallback(HFNWrapper::Status status);

private:
  ros::NodeHandle nh_, pnh_;
  std::string namespace_;
  HFNWrapper* wrapper_;
  std::string action_name_;
  actionlib::SimpleActionServer<hfn::MoveAction> as_;
};

} // end namespace scarab

#endif
