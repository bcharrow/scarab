#include "laser_odom.h"

//ROS======================
ros::NodeHandle *node;
laser_geometry::LaserProjection projector;
sensor_msgs::PointCloud scan_cloud;

//To be published==========
ros::Publisher lodom_pub;
ros::Publisher wodom_pub;
ros::Publisher lscan_pub;
ros::Publisher wscan_pub;
nav_msgs::Odometry laser_state;
nav_msgs::Odometry wheel_state;
sensor_msgs::LaserScan lodom_scan;
sensor_msgs::LaserScan wodom_scan;
tf::TransformBroadcaster* broadcaster;

double freq;
bool publish_map_frame;

//Laser frame offsets======
double laser_xoff;
double laser_yoff;
double laser_zoff;

//Frame IDs================
string lodom_frame_id;
string wodom_frame_id;
string lbase_frame_id;
string wbase_frame_id;
string llaser_frame_id;
string wlaser_frame_id;
string map_frame_id;

//Update Time==============
ros::Time last_update;
ros::Time curr_update;

//For store laser scans====
vector<point2d_t> scan;
vector<point2d_t> tmp_scan;
vector<point2d_t> prev_scan;
vector<scanData> scan_data;

//For laser odometry========
double icp_err;
pose_t curr_laser_twist;
double curr_laser_x = 0;
double curr_laser_y = 0;
double curr_laser_a = 0;
double lodom_cov[36] = {0,   0,   0,        0, 0, 0,  
                        0,   0,   0,        0, 0, 0, 
                        0,   0,   999999,   0, 0, 0,  
                        0,   0,   0,        999999, 0, 0,  
                        0,   0,   0,        0, 999999, 0,  
                        0,   0,   0,        0, 0,      0};  

//For wheel odometry========
pose_t curr_wheel_twist;
double curr_wheel_x = 0;
double curr_wheel_y = 0;
double curr_wheel_a = 0;
double curr_wheel_x_corr = 0;
double curr_wheel_y_corr = 0;
double curr_wheel_a_corr = 0;
double prev_wheel_x = 0;
double prev_wheel_y = 0;
double prev_wheel_a = 0;

void wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  static int init_flag = 0;
  curr_wheel_x = msg->pose.pose.position.x;
  curr_wheel_y = msg->pose.pose.position.y;
  curr_wheel_a = tf::getYaw(msg->pose.pose.orientation);

  if (!init_flag)
  {
    init_flag = 1;
    prev_wheel_x = curr_wheel_x;
    prev_wheel_y = curr_wheel_y;
    prev_wheel_a = curr_wheel_a;
  }
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //Laser Conversion===========================================================  
  projector.projectLaser(*msg, scan_cloud);

  //Convert data to ICP / FIM format========================================
  scan.clear();
  tmp_scan.clear();
  scan_data.clear();
  for (unsigned int i = 0; i < scan_cloud.points.size(); i++)
  {
    point2d_t pt;
    pt.x = scan_cloud.points[i].x;
    pt.y = scan_cloud.points[i].y;
    scan.push_back(pt);

    if ( sqrt(pt.x*pt.x + pt.y*pt.y) < 3.0)
    {
      tmp_scan.push_back(pt);
      //Push scan_data for compute FIM========
      scanData _data;
      _data.pt.x = pt.x;
      _data.pt.y = pt.y;
      _data.r = msg->ranges[scan_cloud.channels[0].values[i]];  //Index in msg
      _data.theta = msg->angle_min + msg->angle_increment*scan_cloud.channels[0].values[i];
      _data.alpha = 100;
      _data.beta = 0;
      scan_data.push_back(_data);
    }
  }

  //Calculate FIM==============================================================
  double cov[3];
  cov_fisher(scan_data, cov);
  if (cov[0] > 0.06)
    cov[0] = 0.06;
  if (cov[0] < 0) {
    ROS_WARN("Returning early");
    //Save Previous Data=========================================================
    prev_scan = scan;
    prev_wheel_x = curr_wheel_x;
    prev_wheel_y = curr_wheel_y;
    prev_wheel_a = curr_wheel_a;
    last_update = curr_update;
    return;
  }

  //Obtain wheel odometry info=================================================
  double diff_wheel_x = curr_wheel_x - prev_wheel_x;
  double diff_wheel_y = curr_wheel_y - prev_wheel_y;
  double diff_wheel_a = curr_wheel_a - prev_wheel_a;
  if (diff_wheel_a > PI) 
    diff_wheel_a -= 2*PI;
  if (diff_wheel_a < -PI) 
    diff_wheel_a += 2*PI;   

  curr_wheel_twist.p.x = diff_wheel_x*cos(prev_wheel_a)+diff_wheel_y*sin(prev_wheel_a);
  curr_wheel_twist.p.y = -diff_wheel_x*sin(prev_wheel_a)+diff_wheel_y*cos(prev_wheel_a);
  curr_wheel_twist.yaw = diff_wheel_a;
   
  //ICP START HERE=============================================================
  if (scan.size() > 0 && prev_scan.size() > 0)
  {
    tree2d_t *tree = tree2d_build(prev_scan.begin(), prev_scan.end(), X, 100);
    pose_t laser_twist_tmp = icp_align(tree, tmp_scan, curr_wheel_twist, 500, 0.001, icp_err);
    if (icp_err < 0.1)
      curr_laser_twist = laser_twist_tmp;
    else
      curr_laser_twist = curr_wheel_twist;
  }

  //Integrate Odometry & Pub===================================================
  //Time===========================================
  curr_update = ros::Time::now();
  double dt = (curr_update-last_update).toSec();
  if (dt > 1)
    dt = 0;
  ros::Time time_stamp = msg->header.stamp;

  //Laser Odometry=================================
  curr_laser_x += curr_laser_twist.p.x*cos(curr_laser_a) - curr_laser_twist.p.y*sin(curr_laser_a);
  curr_laser_y += curr_laser_twist.p.x*sin(curr_laser_a) + curr_laser_twist.p.y*cos(curr_laser_a);
  curr_laser_a += curr_laser_twist.yaw;

  laser_state.header.frame_id = lodom_frame_id;
  laser_state.child_frame_id = lbase_frame_id;
  laser_state.header.stamp = time_stamp;
  laser_state.pose.pose.position.x = curr_laser_x;
  laser_state.pose.pose.position.y = curr_laser_y;
  laser_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(curr_laser_a);
  laser_state.twist.twist.linear.x = curr_laser_twist.p.x / dt;
  laser_state.twist.twist.angular.z = curr_laser_twist.yaw / dt;

  lodom_cov[0]  = cov[0];
  lodom_cov[7]  = cov[1];
  lodom_cov[35] = icp_err;
  for (int i = 0; i < 36; i++)
    laser_state.pose.covariance[i] = lodom_cov[i];

  lodom_pub.publish(laser_state);
  tf::Transform transform_lodom;
  transform_lodom.setOrigin(tf::Vector3(curr_laser_x, curr_laser_y, 0));
  transform_lodom.setRotation(tf::createQuaternionFromYaw(curr_laser_a));
  broadcaster->sendTransform(tf::StampedTransform(transform_lodom, ros::Time(time_stamp), 
                                                  lodom_frame_id, lbase_frame_id));

  //Wheel Odometry=================================
  curr_wheel_x_corr += curr_wheel_twist.p.x*cos(curr_wheel_a_corr) - curr_wheel_twist.p.y*sin(curr_wheel_a_corr);
  curr_wheel_y_corr += curr_wheel_twist.p.x*sin(curr_wheel_a_corr) + curr_wheel_twist.p.y*cos(curr_wheel_a_corr);
  curr_wheel_a_corr += curr_wheel_twist.yaw;

  wheel_state.header.frame_id = wodom_frame_id;
  wheel_state.child_frame_id = wbase_frame_id;
  wheel_state.header.stamp = time_stamp;
  wheel_state.pose.pose.position.x = curr_wheel_x_corr;
  wheel_state.pose.pose.position.y = curr_wheel_y_corr;
  wheel_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(curr_wheel_a_corr);
  wheel_state.twist.twist.linear.x = curr_wheel_twist.p.x / dt;
  wheel_state.twist.twist.angular.z = curr_wheel_twist.yaw / dt;

  wodom_pub.publish(wheel_state);
  tf::Transform transform_wodom;
  transform_wodom.setOrigin(tf::Vector3(curr_wheel_x_corr, curr_wheel_y_corr, 0));
  transform_wodom.setRotation(tf::createQuaternionFromYaw(curr_wheel_a_corr));
  broadcaster->sendTransform(tf::StampedTransform(transform_wodom, ros::Time(time_stamp), 
                                                  wodom_frame_id, wbase_frame_id));

  //Send laser scan & laser frame (On Laser Odometry Frame)=====
  lodom_scan = *msg;
  lodom_scan.header.frame_id = llaser_frame_id;
  lodom_scan.header.stamp = time_stamp;
  lscan_pub.publish(lodom_scan);  
  tf::Transform transform_lscan;
  transform_lscan.setOrigin(tf::Vector3(laser_xoff, laser_yoff, laser_zoff));
  transform_lscan.setRotation(tf::createQuaternionFromYaw(0.0));
  broadcaster->sendTransform(tf::StampedTransform(transform_lscan, ros::Time(time_stamp), 
                                                   lbase_frame_id, llaser_frame_id));

  //Send laser scan & laser frame (On Wheel Odometry Frame)=====
  wodom_scan = *msg;
  wodom_scan.header.frame_id = wlaser_frame_id;
  wodom_scan.header.stamp = time_stamp;
  wscan_pub.publish(wodom_scan);  
  tf::Transform transform_wscan;
  transform_wscan.setOrigin(tf::Vector3(laser_xoff, laser_yoff, laser_zoff));
  transform_wscan.setRotation(tf::createQuaternionFromYaw(0.0));
  broadcaster->sendTransform(tf::StampedTransform(transform_wscan, ros::Time(time_stamp), 
                                                   wbase_frame_id, wlaser_frame_id));

  if (publish_map_frame)  //Send a fake map frame=====
  {
    tf::Transform transform_map;
    transform_map.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform_map.setRotation(tf::createQuaternionFromYaw(0.0));
    broadcaster->sendTransform(tf::StampedTransform(transform_map, ros::Time(time_stamp), 
                                                     map_frame_id, lodom_frame_id));
    broadcaster->sendTransform(tf::StampedTransform(transform_map, ros::Time(time_stamp), 
                                                     map_frame_id, wodom_frame_id));
  }

  //Save Previous Data=========================================================
  prev_scan = scan;
  prev_wheel_x = curr_wheel_x;
  prev_wheel_y = curr_wheel_y;
  prev_wheel_a = curr_wheel_a;
  last_update = curr_update;
}

bool spin()
{
  ros::Rate r(freq);
  while(node->ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_odom");
  ros::NodeHandle n("~");
  node = &n;

  tf::TransformBroadcaster b;
  broadcaster = &b;

  ros::Subscriber sub = n.subscribe("scan",1, scan_callback);
  ros::Subscriber sub1 = n.subscribe("odom",10, wheel_odom_callback);

  lodom_pub = n.advertise<nav_msgs::Odometry>("lodom", 10);
  wodom_pub = n.advertise<nav_msgs::Odometry>("wodom", 10);
  lscan_pub = n.advertise<sensor_msgs::LaserScan>("lscan", 10);
  wscan_pub = n.advertise<sensor_msgs::LaserScan>("wscan", 10);

  node->param("freq", freq, 3.0);
  node->param("publish_map_frame", publish_map_frame, false);

  node->param("laser_xoff", laser_xoff, 0.0);
  node->param("laser_yoff", laser_yoff, 0.0);
  node->param("laser_zoff", laser_zoff, 0.198);

  node->param("lodom_frame", lodom_frame_id, string("lodom"));
  node->param("wodom_frame", wodom_frame_id, string("wodom"));
  node->param("lbase_frame", lbase_frame_id, string("lbase"));
  node->param("wbase_frame", wbase_frame_id, string("wbase"));
  node->param("llaser_frame", llaser_frame_id, string("llaser"));
  node->param("wlaser_frame", wlaser_frame_id, string("wlaser"));
  node->param("map_frame", map_frame_id, string("/map"));

  spin();

  return 0;
}
