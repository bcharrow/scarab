#include <sstream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <nav_msgs/Odometry.h>
#include <odometry_aggregator/OdometryArray.h>

using namespace std;

class OdometrySubscriber
{
public:
  OdometrySubscriber(const ros::NodeHandle &parent, unsigned int id)
  {
    stringstream ss;
    ss << id + 1;
    n = new ros::NodeHandle(parent, "robot" + ss.str());

    sub.subscribe(*n, "odom", 100);
    cache.setCacheSize(1);
    cache.connectInput(sub);
    cache.registerCallback(boost::bind(&OdometrySubscriber::OdometryCallback, this, _1));
  }
  ~OdometrySubscriber() 
  {
    delete n;
  }
  
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odometry = *msg;
    
    return;
  }

  nav_msgs::Odometry* GetOdometry()
  {
    return &odometry;
  }
  
private:
  message_filters::Subscriber<nav_msgs::Odometry> sub;
  message_filters::Cache<nav_msgs::Odometry> cache;

  nav_msgs::Odometry odometry;
  ros::NodeHandle *n;
};

class OdometryAggregator
{
public:
  OdometryAggregator(const ros::NodeHandle &parent, unsigned int count, 
                     const std::string &frame_id)
  {   
    n = new ros::NodeHandle(parent);

    msg.header.frame_id = frame_id;
    msg.array.resize(count);    
    publisher = 
      n->advertise<odometry_aggregator::OdometryArray>("odom_array", 1);

    for (unsigned int i = 0; i < count; i++)
      subscribers.push_back(new OdometrySubscriber(parent, i));
  }
  ~OdometryAggregator() 
  {
    for (vector<OdometrySubscriber*>::iterator i = subscribers.begin();
         i != subscribers.end(); ++i)
      delete *i;

    delete n;
  }

  void Update()
  {
    unsigned int j = 0;
    for (vector<OdometrySubscriber*>::iterator i = subscribers.begin();
         i != subscribers.end(); ++i, j++)
      {
        nav_msgs::Odometry* m = (*i)->GetOdometry();
        msg.array[j] = *m;
      }

    msg.header.stamp = ros::Time::now();
    publisher.publish(msg);
  }

private:  
  ros::NodeHandle *n;
  ros::Publisher publisher;
  odometry_aggregator::OdometryArray msg;
  vector<OdometrySubscriber*> subscribers;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_aggregator");
  ros::NodeHandle n("~");

  int count;
  n.param("count", count, 0);
  assert(count != 0);

  std::string frame_id;
  n.param("frame_id", frame_id, std::string("/map"));
  
  OdometryAggregator a(n, count, frame_id);

  double rate;
  n.param("rate", rate, 10.0);

  ros::Rate r(rate);

  while (n.ok())
    {
      a.Update();

      ros::spinOnce();
      r.sleep();
    }

  return 0;
}
