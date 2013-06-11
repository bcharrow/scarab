#include <string>
#include <vector>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "pose_aggregator/PoseStampedNamed.h"
#include "pose_aggregator/PoseStampedNamedArray.h"

using namespace std;


class PoseSubscriber {
  public:
    PoseSubscriber(const ros::NodeHandle &parent, unsigned int id, 
                   string frame) {
      stringstream ss;
      ss << id;
      node_ = new ros::NodeHandle(parent, "agent" + ss.str());
      sub_ = node_->subscribe("pose", 5, &PoseSubscriber::PoseCallback, this);
      frame_ = frame;
    }

    ~PoseSubscriber() {
      delete node_;
    }

    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
      pose_.header = msg->header;
      pose_.pose = msg->pose;
      pose_.child_frame_id = frame_;
    }

    pose_aggregator::PoseStampedNamed* GetPose() {
      return &pose_;
    }

  private:
    ros::NodeHandle *node_;
    ros::Subscriber sub_;

    string frame_;
    pose_aggregator::PoseStampedNamed pose_;
};


class PoseAggregator {
  public:
    PoseAggregator(const ros::NodeHandle &parent, unsigned int count) {
      node_ = new ros::NodeHandle(parent);

      msg_.poses.resize(count);
      pub_ = node_->advertise<pose_aggregator::PoseStampedNamedArray>(
                              "pose_array", 1);

      for (unsigned int i = 0; i < count; i++) {
        stringstream ss;
        ss << i;
        string frame;
        node_->param("agent" + ss.str() + "_frame", frame, 
                     "agent" + ss.str() + "/base");

        subscribers_.push_back(new PoseSubscriber(parent, i, frame));
      }
    }

    ~PoseAggregator() {
      for (vector<PoseSubscriber*>::iterator i=subscribers_.begin();
           i!=subscribers_.end(); ++i)
        delete *i;

      delete node_;
    }

    void Spin() {
      unsigned int j = 0;
      for (vector<PoseSubscriber*>::iterator i=subscribers_.begin();
           i!=subscribers_.end(); ++i, j++) {
        pose_aggregator::PoseStampedNamed* m = (*i)->GetPose();
        msg_.poses[j] = *m;
      }

      pub_.publish(msg_);
    }

  private:
    ros::NodeHandle *node_;
    ros::Publisher pub_;
    vector<PoseSubscriber*> subscribers_;

    pose_aggregator::PoseStampedNamedArray msg_;
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
