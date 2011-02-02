#include <unistd.h>
#include <string.h>
#include <math.h>

#include "ros/ros.h"

#include "AcronameMotor.h"

#include "DifferentialDriveMsgs/PIDParam.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

#include "acroname_moto/motor_debug.h"

class AcronameMotoDriver
{
private:

  // Params
  double axle_width;
  double max_wheel_vel;
  double min_wheel_vel;
  double wheel_diam;
  double encoder_count_per_motor_rev;
  double motor_to_wheel_ratio;
  double pid_period;
  double pid_param_p, pid_param_i, pid_param_d;
  int left_dir, right_dir;
  std::string portname;
  double freq;

  // For debugging
  acroname_moto::motor_debug debug_data;

  std::string frameid;

  // Static values computed based on params
  double encoder_count_per_meter;
  double meter_per_encoder_count;

  double encvel_per_ms; // (Encoder counts per period) per m/s
  
  ros::NodeHandle *node;
  ros::Publisher odom_pub;
  ros::Publisher debug_pub;

  ros::Subscriber cmd_vel_sub;
  ros::Subscriber param_sub;

  ros::Publisher tuning_pub;

  tf::TransformBroadcaster broadcaster;

  ros::Time last_vel_update;
  nav_msgs::Odometry state;
  double x, y, th, v, w;

  string odom_frame_id;
  string base_frame_id;

  AcronameMotor motor_control;

public:
  AcronameMotoDriver(ros::NodeHandle *node)
  {
    this->node = node;
    odom_pub = node->advertise<nav_msgs::Odometry>("odom", 100);
    debug_pub = node->advertise<acroname_moto::motor_debug>("debug",5);

    tuning_pub = node->advertise<DifferentialDriveMsgs::PIDParam>("tuning_output", 1, true);

    cmd_vel_sub = node->subscribe("cmd_vel", 1, 
                                  &AcronameMotoDriver::OnTwistCmd, this);

    param_sub = node->subscribe("tuning_input", 1,
                                &AcronameMotoDriver::OnParam, this);

    node->param("axle_width", this->axle_width, 0.3);
    node->param("max_wheel_vel", this->max_wheel_vel, 0.5);
    node->param("min_wheel_vel", this->min_wheel_vel, 0.01);
    node->param("frameid", frameid, string("DEFAULT_ODOM"));

    node->param("wheel_diam", this->wheel_diam, 0.1);
    node->param("encoder_count_per_motor_rev", this->encoder_count_per_motor_rev, 16.0);

    node->param("motor_to_wheel_ratio", this->motor_to_wheel_ratio, 19.0);

    node->param("pid_period", this->pid_period, 0.02);
    node->param("pid_param_p", this->pid_param_p, 1.0);
    node->param("pid_param_d", this->pid_param_d, 0.0);
    node->param("pid_param_i", this->pid_param_i, 0.0);
    
    node->param("left_dir", this->left_dir, 1);
    node->param("right_dir", this->right_dir, -1);
    
    int left_channel, right_channel;
    node->param("left_channel", left_channel, 0);
    node->param("right_channel", right_channel, 1);

    node->param("freq", freq, 15.0);

    node->param("portname", this->portname, std::string("acroname"));
    
    encoder_count_per_meter = 
      (encoder_count_per_motor_rev*motor_to_wheel_ratio)/
      (M_PI*wheel_diam);
    meter_per_encoder_count = 1.0/encoder_count_per_meter;

    encvel_per_ms = encoder_count_per_meter*pid_period;

    node->param("odom_frame", odom_frame_id, string("odom"));
    node->param("base_frame", base_frame_id, string("base"));

    int baud;
    node->param("baud", baud, 9600);

    state.header.frame_id = odom_frame_id;
    state.child_frame_id = base_frame_id;

    // odometry starts at zero
    state.pose.pose.position.x = 0.0;
    state.pose.pose.position.y = 0.0;
    state.pose.pose.position.z = 0.0;
    state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    state.twist.twist.linear.x = 0.0;
    state.twist.twist.linear.y = 0.0;
    state.twist.twist.linear.z = 0.0;
    state.twist.twist.angular.x = 0.0;
    state.twist.twist.angular.y = 0.0;
    state.twist.twist.angular.z = 0.0;

    ROS_INFO("Setting up acroname_moto on port %s with baud %d",
             portname.c_str(), baud);
    motor_control.SetupPort(portname, baud);
    motor_control.SetupChannels(left_channel, left_dir, right_channel, right_dir);
    motor_control.SetupPID(pid_param_p, pid_param_i, pid_param_d, pid_period);

    DifferentialDriveMsgs::PIDParam param_msg;
    param_msg.p = pid_param_p;
    param_msg.i = pid_param_i;
    param_msg.d = pid_param_d;
    param_msg.period = pid_period;

    tuning_pub.publish(param_msg);

    this->x = this->y = this->th = this->v = this->w = 0.0;
  }

  bool ok()
  {
    return motor_control.ok();
  }

  void OnTwistCmd(const geometry_msgs::TwistConstPtr &input) 
  {
    ROS_DEBUG("Got cmd_vel: %2.2f %2.2f", input->linear.x, input->angular.z);
    SetVel(input->linear.x, input->angular.z);
  }

  void OnParam(const DifferentialDriveMsgs::PIDParamConstPtr &input) 
  {
    pid_param_p = input->p;
    pid_param_i = input->i;
    pid_param_d = input->d;
    pid_period = input->period;
    
    motor_control.SetupPID(pid_param_p, pid_param_i, pid_param_d, pid_period);

    DifferentialDriveMsgs::PIDParam param_msg;
    param_msg.p = pid_param_p;
    param_msg.i = pid_param_i;
    param_msg.d = pid_param_d;
    param_msg.period = pid_period;

    tuning_pub.publish(param_msg);
  }

  void SetVel(const double &v, const double &w)
  {
    ROS_DEBUG("Recieved velocity command: %f %f", v, w);

    debug_data.sp_v = v;
    debug_data.sp_w = w;

    // Velocity command (v, w) is in this->cmd_vel
    // Compute the differential drive speeds from the input
    double left = v - (axle_width/2.0)*w;
    double right = v + (axle_width/2.0)*w;

    debug_data.sp_wheel_left = left;
    debug_data.sp_wheel_right = right;

    // Scale the speeds to respect the wheel speed limit
    double limitk = 1.0;
    if (fabs(left) > max_wheel_vel)
      limitk = max_wheel_vel/fabs(left);
    if (fabs(right) > max_wheel_vel) {
		double rlimitk = max_wheel_vel/fabs(right);
		if (rlimitk < limitk)
			limitk = rlimitk;
    }
    
    if (limitk != 1.0) {
      left *= limitk;
      right *= limitk;
    }
    
    // Deal with min limits
    if (fabs(left) < min_wheel_vel)
      left = 0.0;
    if (fabs(right) < min_wheel_vel)
      right = 0.0;

    // left and right are wheel speeds in m/s
    // convert to encoder counts per period
    short int enc_left,  enc_right;
    enc_left = round(encvel_per_ms*left);
    enc_right = round(encvel_per_ms*right);

    debug_data.sp_enc_left = enc_left;
    debug_data.sp_enc_right = enc_right;

    int err = motor_control.SetVel(enc_left, enc_right);
    //ROS_INFO("Setting motors: %d %d %d", enc_left, enc_right, err);
  }

  void UpdateVelAndPublish()
  {
    short int enc_left, enc_right;
    motor_control.GetVel(enc_left, enc_right);

    debug_data.enc_left = enc_left;
    debug_data.enc_right = enc_right;

    double left_vel, right_vel;

    left_vel = enc_left/encvel_per_ms;
    right_vel = enc_right/encvel_per_ms;

    debug_data.wheel_left = left_vel;
    debug_data.wheel_right = right_vel;

    this->v = (left_vel + right_vel)/2;
    this->w = (right_vel - left_vel)/axle_width;    

    IntegrateOdometry();

    state.pose.pose.position.x = this->x; 
    state.pose.pose.position.y = this->y;
    state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->th);
    state.twist.twist.linear.x = this->v;
    state.twist.twist.angular.z = this->w;

    debug_data.v = this->v;
    debug_data.w = this->w;

    state.header.stamp = ros::Time::now();
    
    //ROS_INFO("l: %d, r: %d", enc_left, enc_right);

    debug_pub.publish(debug_data);
    odom_pub.publish(state);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(this->x, this->y, 0));
    transform.setRotation(tf::createQuaternionFromYaw(this->th));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(state.header.stamp), 
                                                   odom_frame_id, base_frame_id));    
  }

  void IntegrateOdometry()
  {
    ros::Time now = ros::Time::now();
    double dt = (now-last_vel_update).toSec();

    if(dt > 100.0) {
      last_vel_update = now;
      return;
    }

    // cosine/sine taylor-expanded integrated expression
    double dx,dy,dth;
    dx = this->v*(dt-(this->w*this->w)*(dt*dt*dt)/6.0);
    dy = this->v*(this->w*dt*dt/2.0
                       -(this->w*this->w*this->w)*
                       (dt*dt*dt*dt)/24.0);
    dth = this->w*dt;

    // now add to the current estimate
    this->x += dx*cos(this->th) - dy*sin(this->th);
    this->y += dx*sin(this->th) + dy*cos(this->th);
    this->th += dth;

    last_vel_update = now;
  }

  bool spin()
  {
    ros::Rate r(this->freq);
    while(node->ok()) {
      UpdateVelAndPublish();

      ros::spinOnce();
      r.sleep();
    }

    return true;
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor");
  ros::NodeHandle n("~");
  AcronameMotoDriver d(&n);

  ros::Duration(2.0).sleep();

  if(d.ok()) {
    ROS_INFO("Starting acroname_moto_node::spin()");
    d.spin();
  }
  else {
    ROS_ERROR("Problem starting motor control interface.");
  }

  d.SetVel(0.0, 0.0);

  ros::Duration(2.0).sleep();

  return(0);
}
