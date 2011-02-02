#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped p;
ros::Publisher goal_pub;

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  p = *msg;
  p.header.stamp = ros::Time::now();
  goal_pub.publish(p);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_goal_sync");
  ros::NodeHandle n("~");

  ros::Subscriber goal_sub = n.subscribe("goal_in", 10, goal_callback, ros::TransportHints().udp());
  goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_out",10);
  
  ros::Rate r(10.0);  
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
