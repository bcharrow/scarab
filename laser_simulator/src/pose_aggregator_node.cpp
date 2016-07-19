#include <string>
#include <vector>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "laser_simulator/PoseStampedNamed.h"
#include "laser_simulator/PoseStampedNamedArray.h"

using namespace std;


class PoseSubscriber {
  public:
    PoseSubscriber(const string& agent_prefix, const string& pose_topic,
                   unsigned int id, const string& frame) {
      stringstream ss;
      ss << id;
      node_ = ros::NodeHandle(agent_prefix + ss.str());
      sub_ = node_.subscribe(pose_topic, 5, &PoseSubscriber::PoseCallback, this);
      frame_ = agent_prefix + "/" + frame;
    }

    ~PoseSubscriber() {

    }

    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
      pose_.header = msg->header;
      pose_.pose = msg->pose;
      pose_.child_frame_id = frame_;
    }

    laser_simulator::PoseStampedNamed* GetPose() {
      return &pose_;
    }

  private:
    ros::NodeHandle node_;
    ros::Subscriber sub_;

    string frame_;
    laser_simulator::PoseStampedNamed pose_;
};


class PoseAggregator {
  public:
    PoseAggregator(ros::NodeHandle& node, unsigned int count) {
      msg_.poses.resize(count);
      pub_ = node.advertise<laser_simulator::PoseStampedNamedArray>(
                              "pose_array", 1);

      string agent_prefix;
      node.param("agent_prefix", agent_prefix, string("agent"));

      string pose_topic;
      node.param("pose_topic", pose_topic, string("pose"));

      string frame_id;
      node.param("frame_id", frame_id, string("base"));

      for (unsigned int i = 0; i < count; i++) {
        subscribers_.push_back(new PoseSubscriber(agent_prefix, pose_topic, i, frame_id));
      }
    }

    ~PoseAggregator() {
      for (vector<PoseSubscriber*>::iterator i=subscribers_.begin();
           i!=subscribers_.end(); ++i)
        delete *i;
    }

    void Spin() {
      unsigned int j = 0;
      for (vector<PoseSubscriber*>::iterator i=subscribers_.begin();
           i!=subscribers_.end(); ++i, j++) {
        laser_simulator::PoseStampedNamed* m = (*i)->GetPose();
        msg_.poses[j] = *m;
      }

      pub_.publish(msg_);
    }

  private:
    ros::Publisher pub_;
    vector<PoseSubscriber*> subscribers_;

    laser_simulator::PoseStampedNamedArray msg_;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "PoseAggregator");
  ros::NodeHandle node("~");

  int num_agents;
  node.param("num_agents", num_agents, 0);

  PoseAggregator a(node, num_agents);

  double publish_freq;
  node.param("publish_freq", publish_freq, 5.0);

  ros::Rate r(publish_freq);

  while (node.ok()) {
    a.Spin();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
