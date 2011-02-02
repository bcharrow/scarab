#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;

ros::Publisher pub;

double currX;
double currY;
int currCnt = 0;

double goalArray[100][2] = {82.4326,55.6446,-7.40,5.42};
/*{   19.8556,    4.7107,
   19.9899,   -7.6475,
   34.0944,   -7.7819,
   19.8556,   -7.6475,
   19.8556,    4.5763,
   -5.40  ,    4.67};*/
/*{    1.3621,    4.3000,
    1.5122,   28.0206,
   -7.3455,   28.0206,
    -7.40,      4.67};*/
/*{    1.3621,    4.3000,
    1.3621,   14.6590,
   -7.3455,   14.6590,
   -9.40,     4.67};*/
int goalCnt = 2;

double goalX;
double goalY;

bool init_flag = false;

geometry_msgs::PoseStamped goal;
ros::NodeHandle *node;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  currX = msg->pose.pose.position.x;
  currY = msg->pose.pose.position.y;
  if (!init_flag)
    init_flag = true;
}

void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  if (init_flag && msg->status_list.size() == 0)
  {
    ros::Rate rr(0.3);
    rr.sleep();
    goalX = goalArray[currCnt][0];
    goalY = goalArray[currCnt][1];
    goal.header.frame_id = "map";
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

void setGoal()
{
  if ((goalX-currX)*(goalX-currX)+(goalY-currY)*(goalY-currY) < 1.0*1.0 && init_flag)
  {
    currCnt = currCnt + 1;
    if (currCnt <= goalCnt-1)
    {
      goalX = goalArray[currCnt][0];
      goalY = goalArray[currCnt][1];
      goal.header.frame_id = "map";
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
  }
}

bool spin()
{
  ros::Rate r(10);
  while(node->ok()) 
  {
    setGoal();
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

  ros::Subscriber sub1 = n.subscribe("amcl_pose", 10, pose_callback);
  ros::Subscriber sub3 = n.subscribe("status", 10, status_callback);
  pub = n.advertise<geometry_msgs::PoseStamped>("goal", 100);

  spin();

  return 0;
}
