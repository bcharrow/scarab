#include <string>

#include <ros/ros.h>
#include <mesh80211s/MeshStations.h>

#include "station.h"

int main(int argc, char** argv)
{
  // Setup ROS
  ros::init(argc, argv, "mesh80211s");

  ros::NodeHandle n("~");

  double rate;
  n.param("rate", rate, double(1));

  std::string device;
  n.param("device", device, std::string("mesh"));

  ros::Publisher pub = n.advertise<mesh80211s::MeshStations>("stations", 100);

  mesh80211s::MeshStations msg;

  ros::Rate r(rate);

  struct nl80211_state nlstate;
  struct station_states sstates;

  int err = nl80211_init(&nlstate);
  if (err)
    {
      ROS_INFO("Failed to initialize nlstate");
      return -1;
    }

  while (n.ok())
    {
      err = station_dump(&nlstate, device.c_str());
      if (err < 0)
        {
          ROS_ERROR("station dump failed: %s (%d)\n", strerror(-err), err);
          nl80211_cleanup(&nlstate);
          return -1;
        }

      get_station_states(&sstates);

      msg.stations.resize(sstates.count);

      for (unsigned short i = 0; i < sstates.count; i++)
        {
          msg.stations[i].mac = std::string(sstates.stations[i].mac);
          msg.stations[i].device = std::string(sstates.stations[i].device);
          msg.stations[i].inactive_time = sstates.stations[i].inactive_time;
          msg.stations[i].rx_bytes = sstates.stations[i].rx_bytes;
          msg.stations[i].rx_packets = sstates.stations[i].rx_packets;
          msg.stations[i].tx_bytes = sstates.stations[i].tx_bytes;
          msg.stations[i].tx_packets = sstates.stations[i].tx_packets;
          msg.stations[i].signal = sstates.stations[i].signal;
          msg.stations[i].tx_bitrate = sstates.stations[i].tx_bitrate;
          msg.stations[i].llid = sstates.stations[i].mesh_llid;
          msg.stations[i].plid = sstates.stations[i].mesh_plid;
          msg.stations[i].plink = sstates.stations[i].mesh_plink;
        }

      msg.header.stamp = ros::Time::now();
      pub.publish(msg);

      ros::spinOnce();
      r.sleep();
    }

  nl80211_cleanup(&nlstate);

  return 0;
}
