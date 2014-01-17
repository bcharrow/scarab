#include "ros/ros.h"

#include "hfn.hpp"

using namespace scarab;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hfn");
  ros::NodeHandle nh("~");

  boost::scoped_ptr<HFNWrapper> hfn(HFNWrapper::ROSInit(nh));

  //~ dynamic_reconfigure::Server<human_friendly_navigation::HumanFriendlyNavigationConfig> server;
  //~ server.setCallback(boost::bind(&HumanFriendlyNav::reconfigureCallback, hfn, _1, _2));

  // Wait for connections to form
  ros::Duration(1.0).sleep();
  hfn->stop();
  MoveServer mover("move", hfn.get());

  mover.start();
  ros::spin();
  mover.stop();
  return 0;
}
