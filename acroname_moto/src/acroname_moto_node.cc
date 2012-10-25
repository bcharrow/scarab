#include <unistd.h>
#include <string.h>
#include <math.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"

#include "AcronameMotor.h"

#include "DifferentialDriveMsgs/PIDParam.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

#include "acroname_moto/motor_debug.h"

using namespace std;

class AcronameMotoDriver
{
private:

  // Params
  double axle_width;
  double max_wheel_vel;
  double min_wheel_vel;
  double accel_period_;         /**< Update period of linear ramp */
  double accel_max_;            /**< Maximum acceleration */
  double wheel_diam;
  double encoder_count_per_motor_rev;
  double motor_to_wheel_ratio;
  double pid_period;
  double pid_param_p, pid_param_i, pid_param_d;
  int paramH, paramL;
  int left_dir, right_dir;
  std::string portname;
  double freq;
  AcronameMotor::MotoAddr left_motor, right_motor;

  struct {
    acroname_moto::motor_debug state;
    AcronameMotor control;
  } motors_;
  boost::mutex motors_mutex_; // Guards access to motors

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



public:
  AcronameMotoDriver(ros::NodeHandle *node) {
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


    node->param("accel_max", accel_max_, 0.6);
    node->param("accel_period", accel_period_, 1.0 / 20.0);

    node->param("frameid", frameid, string("DEFAULT_ODOM"));

    node->param("wheel_diam", this->wheel_diam, 0.1);
    node->param("encoder_count_per_motor_rev", this->encoder_count_per_motor_rev, 16.0);

    node->param("motor_to_wheel_ratio", this->motor_to_wheel_ratio, 19.0);

    node->param("pid_period", this->pid_period, 0.02);
    node->param("pid_param_p", this->pid_param_p, 1.0);
    node->param("pid_param_d", this->pid_param_d, 0.0);
    node->param("pid_param_i", this->pid_param_i, 0.0);
    
    node->param("left_dir", this->left_dir, 1);
    node->param("right_dir", this->right_dir, 0);

    node->param("left_module", left_motor.module, 4);
    node->param("right_module", right_motor.module, 4);
    node->param("left_channel", left_motor.channel, 0);
    node->param("right_channel", right_motor.channel, 1);
    
    node->param("paramH", paramH, 0);
    node->param("paramL", paramL, 255);
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
    ROS_INFO("Left Motor: %s  Right Motor: %s",
             left_motor.String().c_str(), right_motor.String().c_str());
    motors_.control.SetupPort(portname, baud);
    motors_.control.SetupChannel(left_motor, left_dir > 0 ? true : false);
    motors_.control.SetupChannel(right_motor, right_dir > 0 ? true : false);
    motors_.control.SetupPID(left_motor, pid_param_p, pid_param_i, pid_param_d,
                           pid_period);
    motors_.control.SetupPID(right_motor, pid_param_p, pid_param_i, pid_param_d,
                           pid_period);
    motors_.control.SetPWMFreq(left_motor, paramH, paramL);
    motors_.control.SetPWMFreq(right_motor, paramH, paramL);

    DifferentialDriveMsgs::PIDParam param_msg;
    param_msg.p = pid_param_p;
    param_msg.i = pid_param_i;
    param_msg.d = pid_param_d;
    param_msg.period = pid_period;

    tuning_pub.publish(param_msg);

    this->x = this->y = this->th = this->v = this->w = 0.0;
  }

  ~AcronameMotoDriver() {
    SetVel(0.0, 0.0);
  }

  bool ok() {
    boost::mutex::scoped_lock lock(motors_mutex_);
    return motors_.control.ok();
  }

  void OnTwistCmd(const geometry_msgs::TwistConstPtr &input)  {
    ROS_DEBUG("Got cmd_vel: %2.2f %2.2f", input->linear.x, input->angular.z);
    SetVel(input->linear.x, input->angular.z);
  }

  void OnParam(const DifferentialDriveMsgs::PIDParamConstPtr &input) {
    pid_param_p = input->p;
    pid_param_i = input->i;
    pid_param_d = input->d;
    pid_period = input->period;
    
    motors_.control.SetupPID(left_motor, pid_param_p, pid_param_i, pid_param_d,
                           pid_period);
    motors_.control.SetupPID(right_motor, pid_param_p, pid_param_i, pid_param_d,
                           pid_period);

    DifferentialDriveMsgs::PIDParam param_msg;
    param_msg.p = pid_param_p;
    param_msg.i = pid_param_i;
    param_msg.d = pid_param_d;
    param_msg.period = pid_period;

    tuning_pub.publish(param_msg);
  }

  // Obtains lock for motor data structure, but should only be called by one
  // thread
  void SetVel(const double &v, const double &w) {
    ROS_DEBUG("Recieved velocity command: %f %f", v, w);

    // Current state
    acroname_moto::motor_debug setpoints;
    double des_left_vel;
    double des_right_vel;
    // Update desired velocity and encoder counts per period
    {
      boost::mutex::scoped_lock lock(motors_mutex_);
      setpoints = motors_.state;

      // Set new setpoints
      motors_.state.sp_v = v;
      motors_.state.sp_w = w;
      // Velocity command (v, w) is in this->cmd_vel
      // Compute the differential drive speeds from the input
      des_left_vel = v - (axle_width/2.0)*w;
      des_right_vel = v + (axle_width/2.0)*w;


      // Scale the speeds to respect the wheel speed limit
      double limitk = 1.0;
      if (fabs(des_left_vel) > max_wheel_vel) {
        limitk = max_wheel_vel/fabs(des_left_vel);
      }
      if (fabs(des_right_vel) > max_wheel_vel) {
        double rlimitk = max_wheel_vel/fabs(des_right_vel);
        if (rlimitk < limitk) {
          limitk = rlimitk;
        }
      }

      if (limitk != 1.0) {
        des_left_vel *= limitk;
        des_right_vel *= limitk;
      }

      // Deal with min limits
      if (fabs(des_left_vel) < min_wheel_vel)
        des_left_vel = 0.0;
      if (fabs(des_right_vel) < min_wheel_vel)
        des_right_vel = 0.0;

      motors_.state.sp_wheel_left = des_left_vel;
      motors_.state.sp_wheel_right = des_right_vel;

      // Convert wheel speeds in in m/s to to encoder counts per period
      motors_.state.sp_enc_left = round(encvel_per_ms *
                                        motors_.state.sp_wheel_left);
      motors_.state.sp_enc_right = round(encvel_per_ms *
                                         motors_.state.sp_wheel_right);

      ROS_DEBUG("Desired encoder setoint: %d %d",
                motors_.state.sp_enc_left, motors_.state.sp_enc_right);
    }

    // Go to new velocity setpoint via linear ramp (i.e. limit max accel)
    ros::Duration d(accel_period_);
    while (true) {
      double left_err = des_left_vel - setpoints.sp_wheel_left;
      double right_err = des_right_vel - setpoints.sp_wheel_right;
      ROS_DEBUG("  Error is %0.3f", left_err);

      double left_corr = copysign(min(fabs(left_err), d.toSec() * accel_max_),
                                  left_err);
      double right_corr = copysign(min(fabs(right_err), d.toSec() * accel_max_),
                                   right_err);
      ROS_DEBUG("  Left Correction=%0.3f  Right correction=%0.3f",
                left_corr, right_corr);

      setpoints.sp_wheel_left += left_corr;
      setpoints.sp_enc_left = round(encvel_per_ms * setpoints.sp_wheel_left);

      setpoints.sp_wheel_right += right_corr;
      setpoints.sp_enc_right = round(encvel_per_ms * setpoints.sp_wheel_right);

      ROS_DEBUG("  Setting motors: %d %d",
                setpoints.sp_enc_left, setpoints.sp_enc_right);

      {
        boost::mutex::scoped_lock lock(motors_mutex_);
        motors_.control.SetVel(left_motor, setpoints.sp_enc_left);
        motors_.control.SetVel(right_motor, setpoints.sp_enc_right);
      }

      if (max(fabs(left_err), fabs(right_err)) == 0) {
        break;
      } else {
        d.sleep();
      }
    }
  }

  // Thread safe way of updating odometry estimate and publishing state
  void UpdateVelAndPublish() {
    short int enc_left, enc_right;
    acroname_moto::motor_debug pub_state;
    {
      boost::mutex::scoped_lock lock(motors_mutex_);
      motors_.control.GetVel(left_motor, &enc_left);
      motors_.control.GetVel(right_motor, &enc_right);

      motors_.state.enc_left = enc_left;
      motors_.state.enc_right = enc_right;

      // ROS_INFO("Motor left = %i Motor right = %i", enc_left, enc_right);
      double left_vel, right_vel;

      left_vel = enc_left/encvel_per_ms;
      right_vel = enc_right/encvel_per_ms;

      motors_.state.wheel_left = left_vel;
      motors_.state.wheel_right = right_vel;

      this->v = (left_vel + right_vel)/2;
      this->w = (right_vel - left_vel)/axle_width;

      motors_.state.v = this->v;
      motors_.state.w = this->w;

      pub_state = motors_.state;
    }
    debug_pub.publish(pub_state);

    IntegrateOdometry();

    state.pose.pose.position.x = this->x;
    state.pose.pose.position.y = this->y;
    state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->th);
    state.twist.twist.linear.x = this->v;
    state.twist.twist.angular.z = this->w;


    state.header.stamp = ros::Time::now();

    //ROS_INFO("l: %d, r: %d", enc_left, enc_right);

    odom_pub.publish(state);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(this->x, this->y, 0));
    transform.setRotation(tf::createQuaternionFromYaw(this->th));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(state.header.stamp),
                                                   odom_frame_id, base_frame_id));
  }

  void IntegrateOdometry() {
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

  void Spin() {
    ros::Rate r(this->freq);
    while(node->ok()) {
      UpdateVelAndPublish();
      r.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor");
  ros::NodeHandle n("~");
  AcronameMotoDriver d(&n);

  ros::Duration(2.0).sleep();

  if(!d.ok()) {
    ROS_ERROR("Problem starting motor control interface.");
    return -1;
  }

  boost::thread motor_thread(&AcronameMotoDriver::Spin, &d);
  ros::spin();

  // Thread should shutdown once ROS is done
  motor_thread.join();

  return 0;
}
