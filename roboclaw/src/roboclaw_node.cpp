#include <unistd.h>
#include <string.h>
#include <math.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

#include "roboclaw/PIDParam.h"
#include "roboclaw/motor_state.h"
#include "RoboClaw.h"

using namespace std;

class DifferentialDriver {
public:
  explicit DifferentialDriver(const ros::NodeHandle &node) : nh_(node) {
    nh_.param("axle_width", axle_width_, 0.275);
    nh_.param("max_wheel_vel", max_wheel_vel_, 0.8);
    nh_.param("min_wheel_vel", min_wheel_vel_, 0.00);
    nh_.param("accel_max", accel_max_, 0.6);
    nh_.param("wheel_diam", wheel_diam_, 0.1);
    nh_.param("quad_pulse_per_motor_rev", quad_pulse_per_motor_rev_, 2000.0);
    nh_.param("motor_to_wheel_ratio", motor_to_wheel_ratio_, 16.0 * 2.375);
    nh_.param("pid_param_p", pid_p_, 0x9000);
    nh_.param("pid_param_i", pid_i_, 0x2000);
    nh_.param("pid_param_d", pid_d_, 0x0000);
    nh_.param("pid_qpps", pid_qpps_, 500000);
    nh_.param("left_sign", left_sign_, -1);
    nh_.param("right_sign", right_sign_, 1);
    nh_.param("portname", portname_, std::string("/dev/roboclaw"));
    nh_.param("baud", baud_, 38400);
    nh_.param("address", address_, 0x80);
    double motor_rev_per_meter = motor_to_wheel_ratio_ / (M_PI * wheel_diam_);
    quad_pulse_per_meter_ = quad_pulse_per_motor_rev_ * motor_rev_per_meter;
    ROS_INFO("qppm: %f", quad_pulse_per_meter_);
    ROS_INFO("Setting up roboclaw_node on port %s with baud %d",
             portname_.c_str(), baud_);
    ser_.reset(new ASIOSerialDevice(portname_.c_str(), baud_));
    claw_.reset(new RoboClaw(ser_.get()));

    // claw_->SetPWM(address_, 0);
    claw_->SetM1Constants(address_, pid_d_, pid_p_, pid_i_, pid_qpps_);
    claw_->SetM2Constants(address_, pid_d_, pid_p_, pid_i_, pid_qpps_);

    accel_max_quad_ = accel_max_ * quad_pulse_per_meter_;

    pub_ = nh_.advertise<roboclaw::motor_state>("motor_state", 5);
  }

  ~DifferentialDriver() {
    try {
      setVel(0.0, 0.0);
    } catch (boost::system::system_error &e) {
      ROS_WARN("Problem turning motors off: %s", e.what());
    }
  }

  void setPID(const roboclaw::PIDParam &param) {
    try {
      claw_->SetM1Constants(address_, param.d, param.p, param.i, param.qpps);
      claw_->SetM2Constants(address_, param.d, param.p, param.i, param.qpps);
    } catch (boost::system::system_error &e) {
      ROS_WARN("Problem setting PID constants: %s", e.what());
    }
  }

  // Convert linear / angular velocity to left / right motor speeds in meters /
  // second
  void vwToWheelSpeed(double v, double w, double *left_mps, double *right_mps) {
    // Compute the differential drive speeds from the input
    *left_mps = v - (axle_width_ / 2.0) * w;
    *right_mps = v + (axle_width_ / 2.0) * w;

    // Scale the speeds to respect the wheel speed limit
    double limitk = 1.0;
    if (fabs(*left_mps) > max_wheel_vel_) {
      limitk = max_wheel_vel_ / fabs(*left_mps);
    }
    if (fabs(*right_mps) > max_wheel_vel_) {
      double rlimitk = max_wheel_vel_ / fabs(*right_mps);
      if (rlimitk < limitk) {
        limitk = rlimitk;
      }
    }

    if (limitk != 1.0) {
      *left_mps *= limitk;
      *right_mps *= limitk;
    }

    // Deal with min limits
    if (fabs(*left_mps) < min_wheel_vel_) {
      *left_mps = 0.0;
    } if (fabs(*right_mps) < min_wheel_vel_) {
      *right_mps = 0.0;
    }

    *right_mps *= right_sign_;
    *left_mps *= left_sign_;
  }

  // Command motors to a given linear and angular velocity
  void setVel(double v, double w) {
    state_.v_sp = v;
    state_.w_sp = w;

    vwToWheelSpeed(v, w, &state_.left_sp, &state_.right_sp);

    // Convert speeds to quad pulses per second
    state_.left_qpps_sp =
      static_cast<int32_t>(round(state_.left_sp * quad_pulse_per_meter_));
    state_.right_qpps_sp =
      static_cast<int32_t>(round(state_.right_sp * quad_pulse_per_meter_));

    ROS_INFO("accel_max_quad %i, left_sp: %i right_sp: %i",
             accel_max_quad_, state_.left_qpps_sp, state_.right_qpps_sp);

    try {
      claw_->SpeedAccelM1(address_, accel_max_quad_, state_.left_qpps_sp);
      claw_->SpeedAccelM2(address_, accel_max_quad_, state_.right_qpps_sp);
    } catch (boost::system::system_error &e) {
      ROS_WARN("Problem setting speed/accel: %s", e.what());
      return;
    }
    ROS_INFO_STREAM("" << state_);
    pub_.publish(state_);
  }

  // Read actual speed of motors and update state
  void update() {
    uint8_t status;
    int32_t speed;
    bool valid;
    try {
      speed = claw_->ReadSpeedM1(address_, &status, &valid);
      if (valid && (status == 0 || status == 1)) {
        state_.left_qpps = speed;
     } else {
        ROS_ERROR("Invalid data from motor 1!");
      }

      speed = claw_->ReadSpeedM2(address_, &status, &valid);
      if (valid && (status == 0 || status == 1)) {
        state_.right_qpps = speed;
      } else {
        ROS_ERROR("Invalid data from motor 2!");
      }
    } catch (boost::system::system_error &e) {
      ROS_WARN("Problem reading speed: %s", e.what());
      return;
    }

    // Convert qpps to meters / second
    state_.right = right_sign_ * state_.right_qpps / quad_pulse_per_meter_;
    state_.left = left_sign_ * state_.left_qpps / quad_pulse_per_meter_;

    state_.v = (state_.right + state_.left) / 2.0;
    state_.w = (state_.right - state_.left) / axle_width_;
    // ROS_INFO_STREAM("" << state_);
    pub_.publish(state_);
  }

  // Get state as reflected by last calls to setVel() and update()
  const roboclaw::motor_state& getState() const {
    return state_;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  boost::scoped_ptr<RoboClaw> claw_;
  boost::scoped_ptr<ASIOSerialDevice> ser_;
  string portname_;
  int baud_;
  int address_;

  double axle_width_;
  double wheel_diam_;
  double motor_to_wheel_ratio_;
  // Max / min velocity of wheels in meters / second
  double min_wheel_vel_, max_wheel_vel_;
  // Maximum wheel acceleration in meters / second^2
  double accel_max_;
  // Quad pulse per second when motor is at 100%
  int pid_qpps_;
  int pid_p_, pid_i_, pid_d_;
  // +1 if positive means forward, -1 if positive means backwards
  int left_sign_, right_sign_;
  // Static values computed based on params
  double quad_pulse_per_motor_rev_;
  double quad_pulse_per_meter_;
  // Max accel in quad pulses per second per second
  uint32_t accel_max_quad_;
  // Current state of motors
  roboclaw::motor_state state_;
};


class RoboClawNode {
public:
  RoboClawNode(ros::NodeHandle *node) : node_(node) {
    odom_pub = node_->advertise<nav_msgs::Odometry>("odom", 100);

    cmd_vel_sub = node_->subscribe("cmd_vel", 1,
                                  &RoboClawNode::OnTwistCmd, this);
    node_->param("odom_frame", odom_state.header.frame_id, string("odom"));
    node_->param("base_frame", odom_state.child_frame_id, string("base"));

    node_->param("freq", freq_, 30.0);

    // Odometry starts at zero
    odom_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    x_ = y_ = th_ = 0.0;

    driver_.reset(new DifferentialDriver(*node_));

    param_sub = node_->subscribe("tuning_input", 1,
                                 &DifferentialDriver::setPID, driver_.get());
  }

  ~RoboClawNode() {
    driver_->setVel(0.0, 0.0);
  }

  void PIDCb(const roboclaw::PIDParam &param) {
    {
      boost::mutex::scoped_lock lock(driver_mutex_);
      driver_->setPID(param);
    }
  }

  // Thread safe way of setting velocity
  void OnTwistCmd(const geometry_msgs::TwistConstPtr &input)  {
    ROS_DEBUG("Got cmd_vel: %2.2f %2.2f", input->linear.x, input->angular.z);
    {
      boost::mutex::scoped_lock lock(driver_mutex_);
      driver_->setVel(input->linear.x, input->angular.z);
    }
  }

  // Thread safe way of updating odometry estimate and publishing state
  void UpdateVelAndPublish() {
    roboclaw::motor_state state;
    {
      boost::mutex::scoped_lock lock(driver_mutex_);
      driver_->update();
      state = driver_->getState();
    }

    IntegrateOdometry(state);

    odom_state.pose.pose.position.x = x_;
    odom_state.pose.pose.position.y = y_;
    odom_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);
    odom_state.twist.twist.linear.x = state.v;
    odom_state.twist.twist.angular.z = state.w;
    odom_state.header.stamp = ros::Time::now();

    odom_pub.publish(odom_state);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_, y_, 0));
    transform.setRotation(tf::createQuaternionFromYaw(th_));
    broadcaster.sendTransform(tf::StampedTransform(transform,
                                                   ros::Time(odom_state.header.stamp),
                                                   odom_state.header.frame_id,
                                                   odom_state.child_frame_id));
  }

  // Integrate odometry given motor's current speed
  void IntegrateOdometry(const roboclaw::motor_state &state) {
    ros::Time now = ros::Time::now();
    double dt = (now-last_vel_update).toSec();

    if(dt > 10.0) {
      last_vel_update = now;
      return;
    }

    // cosine/sine taylor-expanded integrated expression
    double dx,dy,dth;
    dx = state.v * (dt - (state.w*state.w) * (dt*dt*dt) / 6.0);
    dy = state.v * (state.w*dt*dt/2.0
                    -(state.w*state.w*state.w)*
                    (dt*dt*dt*dt)/24.0);
    dth = state.w * dt;

    // now add to the current estimate
    x_ += dx*cos(th_) - dy*sin(th_);
    y_ += dx*sin(th_) + dy*cos(th_);
    th_ += dth;

    last_vel_update = now;
  }

  void Spin() {
    ros::Rate r(freq_);
    while (node_->ok()) {
      UpdateVelAndPublish();
      r.sleep();
    }
  }

private:
  // Motor parameters
  boost::scoped_ptr<DifferentialDriver> driver_;
  boost::mutex driver_mutex_; // Guards access to motors

  // How frequently to read speed and update odometry
  double freq_;

  // Current x, y, theta estimate given odometry
  ros::Time last_vel_update;
  nav_msgs::Odometry odom_state;
  double x_, y_, th_;

  ros::NodeHandle *node_;
  ros::Publisher odom_pub;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber param_sub;
  tf::TransformBroadcaster broadcaster;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor");
  ros::NodeHandle nh("~");
  RoboClawNode rcn(&nh);

  ros::Duration(2.0).sleep();

  boost::thread motor_thread(&RoboClawNode::Spin, &rcn);
  ros::spin();

  // Thread should shutdown once ROS is done
  motor_thread.join();

  return 0;
}
