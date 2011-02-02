#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;

ros::Publisher pub;

double goalX;
double goalY;

bool init_flag = false;

geometry_msgs::PoseStamped goal;
ros::NodeHandle *node;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if (!init_flag)
    init_flag = true;
}

std::string goal_frame_id;

void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  if (init_flag && msg->status_list.size() == 0)
  {
    ros::Rate rr(0.3);
    rr.sleep();
    goal.header.frame_id = goal_frame_id;
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = goalX;
    goal.pose.position.y = goalY;
    goal.pose.position.z = 0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 1;
    goal.pose.orientation.w = 0;
    pub.publish(goal);
  }
  else if (init_flag && msg->status_list.size() == 1 && msg->status_list[0].status == 4)
  {
    cout <<"Goal Failed"<<endl;
    ros::Rate rr(1);
    goal.header.stamp = ros::Time::now();
    pub.publish(goal);
    rr.sleep();
  }
}

bool spin()
{
  ros::Rate r(10);
  while(node->ok()) 
  {
    ros::spinOnce();
    r.sleep();
  }
  return true;
}

int main(int argc, char** argv)
{
  // Setup ROS
  ros::init(argc, argv, "simple_nav");
  ros::NodeHandle n("~");
  node = &n;

  n.param("goal_frame_id", goal_frame_id, std::string("/map"));

  node->param("goto_x",goalX , -7.40);
  node->param("goto_y",goalY , 5.42);

  ros::Subscriber sub1 = n.subscribe("amcl_pose", 10, pose_callback);
  ros::Subscriber sub3 = n.subscribe("status", 10, status_callback);
  pub = n.advertise<geometry_msgs::PoseStamped>("goal", 100);

  spin();

  return 0;
}
