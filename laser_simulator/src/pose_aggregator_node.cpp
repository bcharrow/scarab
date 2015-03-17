#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <laser_simulator/PoseStampedNamedArray.h>

#include <boost/thread/mutex.hpp>

using namespace std;


class PoseSubscriber {
private:
  ros::Subscriber sub_;

  boost::mutex pose_mutex_;

  string name_;
  string frame_;
  laser_simulator::PoseStampedNamed pose_;

  bool active_;

public:
  PoseSubscriber(ros::NodeHandle *node, string name,
                 string frame, string pose_topic) :
                 name_(name), frame_(frame), active_(false) {
    sub_ = node->subscribe(string("/") + name + string("/") + pose_topic,
                    5, &PoseSubscriber::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(pose_mutex_);
    active_ = true;

    pose_.header = msg->header;
    pose_.pose = msg->pose;
    pose_.child_frame_id = string("/") + name_ + string("/") + frame_;
  }

  bool getPose(laser_simulator::PoseStampedNamed* pose) {
    boost::mutex::scoped_lock lock(pose_mutex_);
    *pose = pose_;
    return sub_.getNumPublishers() > 0;
  }

  string getName(void) { return name_; }

  int getNumPublishers(void) { return sub_.getNumPublishers(); }

  bool active(void) { return active_; }
};


class PoseAggregator {
private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  map<string, PoseSubscriber *> subscribers_;
  int max_num_agents_;

  string agent_prefix_;
  string frame_id_;
  string pose_topic_;

public:
  PoseAggregator(ros::NodeHandle &node) : n_() {
    int num_agents;
    string agent_name, frame_id, pose_topic;
    node.param("agent_prefix", agent_prefix_, string("scarab"));
    node.param("num_agents", num_agents, 0);
    node.param("frame_id", frame_id_, string("base_link"));
    node.param("pose_topic", pose_topic_, string("pose"));
    max_num_agents_ = num_agents;

    for (unsigned int i = 0; i <= num_agents; ++i) {
      stringstream name;
      name << agent_prefix_ << i;

      subscribers_[name.str()] = new PoseSubscriber(&n_, name.str(), frame_id_, pose_topic_);
    }

    pub_ = n_.advertise<laser_simulator::PoseStampedNamedArray>(
            "pose_array", 1);
  }

  ~PoseAggregator() {
    for (map<string, PoseSubscriber*>::iterator it = subscribers_.begin();
         it != subscribers_.end(); ++it) {
      delete it->second;
    }
  }

  void Spin() {
    laser_simulator::PoseStampedNamedArray msg;

    for (map<string, PoseSubscriber*>::iterator it = subscribers_.begin();
         it != subscribers_.end(); ++it) {
      if (!it->second->active()) {
        continue;
      }
      laser_simulator::PoseStampedNamed p;
      if (it->second->getPose(&p)) {
        msg.poses.push_back(p);
      } else {
        stringstream name;
        name << agent_prefix_ << max_num_agents_;
        if (name.str() != it->second->getName()) {
          delete it->second;
          subscribers_.erase(it);
        }
      }
    }

    while (true) {
      stringstream name;
      name << agent_prefix_ << max_num_agents_;
      if (subscribers_[name.str()]->getNumPublishers() > 0) {
        stringstream new_name;
        new_name << agent_prefix_ << ++max_num_agents_;
        subscribers_[new_name.str()] = new PoseSubscriber(&n_, new_name.str(), frame_id_, pose_topic_);
      } else {
        break;
      }
    }

    pub_.publish(msg);
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "PoseAggregator");
  ros::NodeHandle node("~");

  double publish_freq;
  node.param("publish_freq", publish_freq, 5.0);
  ros::Rate r(publish_freq);

  PoseAggregator a(node);

  while (node.ok()) {
    a.Spin();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
