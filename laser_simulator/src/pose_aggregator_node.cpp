#include <string>
#include <vector>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "laser_simulator/PoseStampedNamed.h"
#include "laser_simulator/PoseStampedNamedArray.h"

#include <boost/thread/mutex.hpp>

using namespace std;


class PoseSubscriber {
  private:
    ros::Subscriber sub_;

    boost::mutex pose_mutex_;

    string name_;
    string frame_;
    laser_simulator::PoseStampedNamed pose_;

  public:
    PoseSubscriber(ros::NodeHandle *node, string name,
                   string frame, string pose_topic) :
                   name_(name), frame_(frame) {
      sub_ = node->subscribe(string("/") + name + string("/") + pose_topic,
                      5, &PoseSubscriber::PoseCallback, this);
    }

    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
      boost::mutex::scoped_lock lock(pose_mutex_);
      pose_.header = msg->header;
      pose_.pose = msg->pose;
      pose_.child_frame_id = string("/") + name_ + string("/") + frame_;
    }

    laser_simulator::PoseStampedNamed GetPose() {
      boost::mutex::scoped_lock lock(pose_mutex_);
      return pose_;
    }
};


class PoseAggregator {
  private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    vector<PoseSubscriber *> subscribers_;

    laser_simulator::PoseStampedNamedArray msg_;

  public:
    PoseAggregator(ros::NodeHandle &node) : n_() {
      int num_agents;
      string agent_name, frame_id, pose_topic;
      node.param("agent_prefix", agent_name, string("scarab"));
      node.param("num_agents", num_agents, 0);
      node.param("frame_id", frame_id, string("base_link"));
      node.param("pose_topic", pose_topic, string("pose"));

      msg_.poses.resize(num_agents);

      for (unsigned int i=0; i<num_agents; ++i) {
        stringstream name;
        name << agent_name << i;

        subscribers_.push_back(new PoseSubscriber(&n_, name.str(), frame_id, pose_topic));
      }

      pub_ = n_.advertise<laser_simulator::PoseStampedNamedArray>(
              "pose_array", 1);
    }

    ~PoseAggregator() {
      for (vector<PoseSubscriber*>::iterator it=subscribers_.begin();
           it!=subscribers_.end(); ++it) {
        delete *it;
      }
    }

    void Spin() {
      unsigned int j = 0;
      for (vector<PoseSubscriber*>::iterator i=subscribers_.begin();
           i!=subscribers_.end(); ++i, j++) {
        msg_.poses[j] = (*i)->GetPose();
      }

      pub_.publish(msg_);
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
