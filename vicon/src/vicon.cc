#include <ros/ros.h>

#include <string>
#include <vector>
#include <vicon/ViconState.h>

#include <nmw/Config.h>
#include "StateEstimation.h"
#include "ViconSDK.h"
#include "ViconSubject.h"
#include "QuatToRPY.h"
#include <VelocityEstimator.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon");

  ros::NodeHandle n("~");

  std::string host;
  n.param("host", host, std::string("alkaline"));

  int sd = CreateTCPSocket((char *)host.c_str());

  if(sd < 0) 
    {
      ROS_ERROR("Failed to create tcp socket to %s\n", host.c_str());
      return -1;
    }

  double rate;
  n.param("rate", rate, double(100));
  ros::Rate r(rate);

  ros::Publisher pub = n.advertise<vicon::ViconState>("state", 100);
  vicon::ViconState msg;

  /* State Variables */
  std::vector< std::string > info;
  std::vector< MarkerChannel > MarkerChannels;
  std::vector< BodyChannel > BodyChannels;
  std::vector< MarkerData > markerPositions;
  std::vector< BodyData > bodyPositions;
  int FrameChannel;
  double timestamp;
  std::vector<double> data;
  std::map<std::string, int> fiducial_mapping;
  std::map<std::string, StateEstimate> state_estimates;
  std::map<std::string, ViconSubject> viconSubjects;
  std::map<std::string, VelocityEstimator> velocity_estimates;
  double fps;

  std::string configfile;
  n.param("configfile", configfile, std::string("config.xml"));

  nmw::Config *c = new nmw::Config(configfile.c_str());
  if(c->Load() != 0)
    return -1;
  
  std::string vskpath = configfile.substr(0, configfile.rfind("/")) + std::string("/vsk/");   

  ROS_INFO("Load Subject Information");
  if(ReadXmlConfig(*c, fiducial_mapping, 
                   state_estimates, 
                   viconSubjects, vskpath) < 0)
    return 2;

  GetViconInfo(sd, info);
  ParseViconInfo(info, MarkerChannels, BodyChannels, FrameChannel, fps);
  RequestViconStream(sd);

  bool msg_resize = false;

  while (n.ok())
    {     
      GetViconData(sd, info, data);
      ProcessViconData(data, MarkerChannels, BodyChannels, FrameChannel,
                       markerPositions, bodyPositions, timestamp, 
                       state_estimates, viconSubjects);

      std::map<std::string, StateEstimate>::iterator iter;
      if (!msg_resize)
        {
          int count = 0;
          for(iter = state_estimates.begin(); iter != state_estimates.end(); ++iter)
            if (iter->second.tracking)
              count++;
          msg.subject.resize(count);
          msg_resize = true;
        }
      
      ros::Time now = ros::Time::now();

      int i = 0;
      for(iter = state_estimates.begin(); iter != state_estimates.end(); ++iter)
        {
          if(iter->second.tracking)
            {
              double roll, pitch, yaw;
              double xdot, ydot, zdot, rolldot, pitchdot, yawdot;

              QuatToRPY(iter->second.robotBodyEstimate.Q.x,
                        iter->second.robotBodyEstimate.Q.y,
                        iter->second.robotBodyEstimate.Q.z,
                        iter->second.robotBodyEstimate.Q.w,
                        roll, pitch, yaw);
              
              velocity_estimates[iter->first].SetPose(now.toSec(),
                                                      iter->second.robotBodyEstimate.TX/1000.0, 
                                                      iter->second.robotBodyEstimate.TY/1000.0, 
                                                      iter->second.robotBodyEstimate.TZ/1000.0,
                                                      roll, pitch, yaw);
              
              velocity_estimates[iter->first].GetVelocity(xdot, ydot, zdot, 
                                                          rolldot, pitchdot, yawdot);
              
              msg.subject[i].name = iter->first;

              msg.subject[i].pose.position.x = iter->second.robotBodyEstimate.TX/1000.0;
              msg.subject[i].pose.position.y = iter->second.robotBodyEstimate.TY/1000.0;
              msg.subject[i].pose.position.z = iter->second.robotBodyEstimate.TZ/1000.0;

              msg.subject[i].pose.orientation.x = iter->second.robotBodyEstimate.Q.x;
              msg.subject[i].pose.orientation.y = iter->second.robotBodyEstimate.Q.y;
              msg.subject[i].pose.orientation.z = iter->second.robotBodyEstimate.Q.z;
              msg.subject[i].pose.orientation.w = iter->second.robotBodyEstimate.Q.w;
              
              msg.subject[i].vel.linear.x = xdot;
              msg.subject[i].vel.linear.y = ydot;
              msg.subject[i].vel.linear.z = zdot;
              msg.subject[i].vel.angular.x = rolldot;
              msg.subject[i].vel.angular.y = pitchdot;
              msg.subject[i].vel.angular.z = yawdot;

              i++;
            }
        }

      msg.header.stamp = now;
      pub.publish(msg);

      ros::spinOnce();
      r.sleep();
    }

  return 0;
}
