/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**

@mainpage

@htmlinclude manifest.html

@b urg_node is a driver for the Hokuyo URG-04LX

<hr>

@section information Information

Hokuyo scans are taken in a counter-clockwise direction.  Angles are measured
counter clockwise with 0 pointing directly forward.

<hr>

@section usage Usage
@verbatim
$ urg_node [standard ROS args]
@endverbatim

@par Example

@verbatim
$ urg_node
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "scan"/<a href="../../laser_scan/html/classstd__msgs_1_1LaserScan.html">laser_scan/LaserScan</a> : scan data from the laser.
- @b "/diagnostics"/<a href="../../robot_msgs/html/classrobot__msgs_1_1DiagnosticMessage.html">robot_msgs/DiagnosticMessage</a> : diagnostic status information.

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "~min_ang_degrees" : @b [double] the angle of the first range measurement in degrees (Default: -120.0)
- @b "~max_ang_degrees" : @b [double] the angle of the last range measurement in degrees (Default: 120.0)
- @b "~min_ang"         : @b [double] the angle of the first range measurement in radians (Default: -2*pi/3)
- @b "~max_ang"         : @b [double] the angle of the last range measurement in radians (Default: 2*pi/3)
- @b "~use_intensity"       : @b [bool]   whether or not the hokuyo returns intensity values (Default: false)
- @b "~skip"            : @b [int]    the number of scans to skip between each measured scan (Default: 1)
- @b "~port"            : @b [string] the port where the hokuyo device can be found (Default: "/dev/urg")
- @b "~autostart"       : @b [bool]   whether the node should automatically start the hokuyo (Default: true)
- @b "~frame_id"        : @b [string] the frame in which laser scans will be returned (Default: "FRAMEID_LASER")
 **/

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"

//#include "self_test/self_test.h"

#include "Hokuyo.hh"

#include "tf/transform_broadcaster.h"

#define URG04_MIN_STEP 44
#define URG04_MAX_STEP 725

using namespace std;

class UrgNode//: public ros::Node
{
private:
  bool running_;

  int count_;

  ros::NodeHandle node_;
  ros::Publisher scan_pub;

public:
  //hokuyo::Laser laser_;
  Hokuyo laser_;
  sensor_msgs::LaserScan scan_msg_;
  tf::TransformBroadcaster* broadcaster;
  tf::Transform pose;

  double min_ang_;
  double max_ang_;
  double min_range_;
  double max_range_;
  bool intensity_;
  int cluster_;
  int skip_;
  string port_;
  bool autostart_;
  bool calibrate_time_;
  bool use_intensity_;
  string laser_frame_id;
  string parent_frame_id;
  string device_id_;
  string device_status_;
  string connect_fail_;
  int baud_rate_;
  int sensor_type;
  int count_zero;

  double rate;

  // Internal Laser Params
  double resolution;
  unsigned int * laser_data;

  UrgNode(ros::NodeHandle &node) : running_(false), count_(0)
  {

    std::string type;
    node_.param("type", type, std::string("urg"));

    node_.param("rate", rate, 10.0);

    if (type == std::string("urg"))
      laser_.SetType(HOKUYO_TYPE_URG_04LX);
    else
      laser_.SetType(HOKUYO_TYPE_UTM_30LX);

    this->node_ = node;
    scan_pub = node_.advertise<sensor_msgs::LaserScan>("scan", 100);

    if (node_.hasParam("min_ang_degrees") && node_.hasParam("min_ang"))
    {
      ROS_FATAL("Minimum angle is specified in both radians and degrees");
      node_.shutdown();
    }

    if (node_.hasParam("max_ang_degrees") && node_.hasParam("max_ang"))
    {
      ROS_FATAL("Maximum angle is specified in both radians and degrees");
      node_.shutdown();
    }

    if (node_.hasParam("min_ang_degrees"))
    {
      node_.getParam("min_ang_degrees", min_ang_);
      min_ang_ *= M_PI/180;
    }
    else if (node_.hasParam("min_ang"))
    {
      node_.getParam("min_ang", min_ang_);
    }
    else
    {
      min_ang_ = -2*M_PI/3.0;
    }

    if (node_.hasParam("max_ang_degrees"))
    {
      node_.getParam("max_ang_degrees", max_ang_);
      max_ang_ *= M_PI/180;
    }
    else if (node_.hasParam("max_ang"))
    {
      node_.getParam("max_ang", max_ang_);
    }
    else
    {
      max_ang_ = 2*M_PI/3.0;
    }

    node_.param("cluster", cluster_, 1);
    node_.param("skip", skip_, 1);
    node_.param("port", port_, string("/dev/urg"));
    node_.param("autostart", autostart_, true);
    node_.param("calibrate_time", calibrate_time_, true);
    node_.param("laser_frame", laser_frame_id, string("laser"));
    node_.param("parent_frame", parent_frame_id, string("base"));
    node_.param("baud", baud_rate_, 115200);
    node_.param("use_intensity", use_intensity_, false);

    /*
    string path, prefix;
    bool use_prefix = false;
    if (node_.searchParam(string("tf_prefix"), path))
      {
        node_.getParam(path, prefix);
        use_prefix = true;
      }

    if (use_prefix)
      {
        parent_frame_id = tf::remap(prefix, parent_frame_id);
        laser_frame_id = tf::remap(prefix, laser_frame_id);
      }
    */

    laser_data = new unsigned int[HOKUYO_MAX_DATA_LENGTH];
    assert (laser_data);
  }

  ~UrgNode()
  {
    stop();
    delete laser_data;
    delete broadcaster;
  }

  int start()
  {
    stop();

    /*
    try
    {
    */
      device_id_ = std::string("unknown");
      device_status_ = std::string("unknown");

      if( laser_.Connect(port_.c_str(), baud_rate_) < 0 ) {
        ROS_WARN("Problem connecting to hokuyo laser...\n");
        return -1;
      }

      ROS_INFO("Connected to URG device\n");

      //correct the min/max angles if needed
      float laser_min_angle = laser_.GetAngleMin();
      float laser_max_angle = laser_.GetAngleMax();
      float laser_res        = laser_.GetAngleRes();
      count_zero            = laser_.GetCountZero();

      //first limit the angles to the sensor capabilities
      min_ang_ = min_ang_ >= laser_min_angle ? min_ang_ : laser_min_angle;
      max_ang_ = max_ang_ <= laser_max_angle ? max_ang_ : laser_max_angle;     

      resolution = (double)laser_res;
      
      //get range info from the sensor
      max_range_ = laser_.GetDistMax();
      min_range_ = 0.05;

      node_.setParam("min_ang_limit", min_ang_);
      node_.setParam("max_ang_limit", max_ang_);
      node_.setParam("min_range", min_range_);
      node_.setParam("max_range", max_range_);
      
      //cfg_.scan_time = 1.0/laser_.GetScanRate();
      
      sensor_type = laser_.GetSensorType();
      
      if (use_intensity_ == true)
        {
          //URG fixes resolution at 1/3 of normal when requesting rand + intensity
          if (sensor_type == HOKUYO_TYPE_URG_04LX)
            {    
              skip_ = 3;
            }
        }

      //Conf.resolution= laser_res * skip_;

      //correct for the edge effects if the skip_ value is not 1
      int exp_num_points = ceil( (max_ang_ - min_ang_ + laser_res) / ( skip_ * laser_res ) );
      max_ang_ = exp_num_points*skip_*laser_res - laser_res + min_ang_;
         
      double x, y, z, roll, pitch, yaw;
      
      node_.param("laser_transform/x", x, 0.0);
      node_.param("laser_transform/y", y, 0.0);
      node_.param("laser_transform/z", z, 0.0);
      node_.param("laser_transform/roll", roll, 0.0);
      node_.param("laser_transform/pitch", pitch, 0.0);
      node_.param("laser_transform/yaw", yaw, 0.0);
      
      tf::Vector3 p(x, y, z);
      tf::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      
      pose.setOrigin(p);
      pose.setRotation(q);
      
      broadcaster = new tf::TransformBroadcaster;

      running_ = true;

    return(0);
  }

  int stop()
  {
    if(running_)
    {
        laser_.Disconnect();

        running_ = false;
    }

    return 0;
  }

  int publishScan()
  {


    // update device data
    //Laser.GetReadings (Readings, min_i, max_i);
    int n_data;
    
    //calculate the start and stop angle counts + round
    int scan_start = min_ang_ > 0 ? 
      (int)floor(min_ang_ / resolution * skip_ + count_zero) :
      (int)ceil(min_ang_ / resolution * skip_ + count_zero);
    int scan_end = max_ang_ > 0 ?
      (int)floor( max_ang_ / resolution * skip_ + count_zero) :
      (int)ceil( max_ang_ / resolution * skip_ + count_zero);

    double min_angle = (scan_start - count_zero)*resolution;
    double max_angle = (scan_end - count_zero)*resolution;
    
    ROS_DEBUG("Scan Start: %d, end: %d", scan_start, scan_end);
    
    int ret;

    //request just ranges
    if (use_intensity_ == false )
    {
      ret = laser_.GetScan(laser_data, n_data, scan_start, scan_end, 
                skip_, HOKUYO_3DIGITS, HOKUYO_SCAN_REGULAR,0);

      if ( ret == 0)    //successful scan
      {

        scan_msg_.ranges.resize(n_data);
        scan_msg_.intensities.resize(0);

        scan_msg_.angle_min = min_angle;
        scan_msg_.angle_max = max_angle;
        scan_msg_.angle_increment = resolution;
        scan_msg_.time_increment = 0; // FIXME: this is a dummy value
        scan_msg_.scan_time = 0; // FIXME: this is a dummy value
        scan_msg_.range_min = min_range_;
        scan_msg_.range_max = max_range_;
        
        for (int i = 0; i < n_data; i++)
        {
          scan_msg_.ranges[i]  = laser_data[i]/1000.0;

          if(scan_msg_.ranges[i] < min_range_)
            scan_msg_.ranges[i] = max_range_;
        }

        scan_msg_.header.stamp = ros::Time::now();
        scan_msg_.header.frame_id = laser_frame_id;

        ROS_DEBUG("Publishing %d ranges\n", (int)scan_msg_.ranges.size());

        scan_pub.publish (scan_msg_);

        broadcaster->sendTransform(tf::StampedTransform(pose, ros::Time::now(),
                                                        parent_frame_id, laser_frame_id));  
      }
    }

    //get the ranges and intensities
    else
    {
      if (sensor_type == HOKUYO_TYPE_URG_04LX)
      {
        ret = laser_.GetScan(laser_data, n_data, scan_start, scan_end, 
                HOKUYO_RANGE_INTENSITY_AV_AGC_AV, HOKUYO_3DIGITS, HOKUYO_SCAN_REGULAR,0);
        
        if ( ret == 0)  //successful scan
        {

          scan_msg_.ranges.resize(n_data/3);
          scan_msg_.intensities.resize(n_data/3);

          scan_msg_.angle_min = min_ang_;
          scan_msg_.angle_max = max_ang_;
          scan_msg_.angle_increment = resolution;
          scan_msg_.time_increment = 0; // FIXME: this is a dummy value
          scan_msg_.scan_time = 0; // FIXME: this is a dummy value
          scan_msg_.range_min = min_range_;
          scan_msg_.range_max = max_range_;
          

          //for URG 04-LX, this specific packet contains ranges, intensities, and AGC values
          //AGC= Auto Gain Control. Intensity values are not very useful, since they jump around a lot
          //AGC represents relative intensity better as it is a coarse components of the intensity
          //Where the intensity value, returned by the sensor is the fine component.
          
          for (int i = 0; i < n_data/3; i++)
          {
            scan_msg_.ranges[i]     = laser_data[3*i]/1000.0;
            scan_msg_.intensities[i]  = (uint8_t)(laser_data[3*i+2]/4); //FIXME: use the range of values better

            if(scan_msg_.ranges[i] < min_range_)
              scan_msg_.ranges[i] = max_range_;
          }

          scan_msg_.header.stamp = ros::Time::now();//.fromNSec((uint64_t)scan_.system_time_stamp);
          scan_msg_.header.frame_id = laser_frame_id;
          
          scan_pub.publish(scan_msg_);

          broadcaster->sendTransform(tf::StampedTransform(pose, ros::Time::now(),
                                                  parent_frame_id, laser_frame_id));  
  
        }
      }

      else if (sensor_type == HOKUYO_TYPE_UTM_30LX)
	{
        ret = laser_.GetScan(laser_data, n_data, scan_start, scan_end, 
                skip_, HOKUYO_3DIGITS, HOKUYO_SCAN_SPECIAL_ME,0);
        
        if ( ret == 0)  //successful scan
        {

          scan_msg_.ranges.resize(n_data/2);
          scan_msg_.intensities.resize(n_data/2);

          scan_msg_.angle_min = min_ang_;
          scan_msg_.angle_max = max_ang_;
          scan_msg_.angle_increment = resolution;
          scan_msg_.time_increment = 0; // FIXME: this is a dummy value
          scan_msg_.scan_time = 0; // FIXME: this is a dummy value
          scan_msg_.range_min = min_range_;
          scan_msg_.range_max = max_range_;
          

          for (int i = 0; i < n_data/2; i++)
          {
            scan_msg_.ranges[i]     = laser_data[2*i]/1000.0;
            scan_msg_.intensities[i]  = (uint8_t)(laser_data[2*i+1]/40); //FIXME: use the range of values better

            if(scan_msg_.ranges[i] < min_range_)
              scan_msg_.ranges[i] = max_range_;
          }

          scan_msg_.header.stamp = ros::Time::now();//.fromNSec((uint64_t)scan_.system_time_stamp);
          scan_msg_.header.frame_id = laser_frame_id;

          scan_pub.publish(scan_msg_);

          broadcaster->sendTransform(tf::StampedTransform(pose, ros::Time::now(),
                                                          parent_frame_id, laser_frame_id));  
  
        }
      }
    }


    count_++;

    return(0);
  }

  bool spin()
  {
    ros::Rate r(rate);
    // Start up the laser
    while (node_.ok())
      {
        if (autostart_ && start() == 0)
          while(node_.ok()) 
            if(publishScan() < 0)
              break;
       
        ros::spinOnce();
        r.sleep();        
      }
  
    //stopping should be fine even if not running
    stop();

    return true;
  }
};


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "urg");

  ros::NodeHandle n("~");
  UrgNode h(n);

  h.spin();

  return(0);
}
