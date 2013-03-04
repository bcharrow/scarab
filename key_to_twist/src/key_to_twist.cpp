#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <curses.h>

using namespace std;

int main(int argc, char** argv)
{
    std::string output_topic;
    double v_speed, v_inc;
    double w_speed, w_inc;

    geometry_msgs::Twist twist;
    ros::Publisher pub;
    ros::NodeHandle *node;

    ros::init(argc, argv, "key_to_twist");
    ros::NodeHandle n("~");
    node = &n;

    node->param("vspeed", v_speed, 0.6);
    node->param("wspeed", w_speed, 0.6);
    node->param("vinc", v_inc, 0.08);
    node->param("winc", w_inc, 0.05);
    node->param("output", output_topic, std::string("/motor/cmd_vel"));

    pub = n.advertise<geometry_msgs::Twist>(output_topic, 10);

    initscr();
    clear();
    noecho();
    cbreak();
    timeout(50); // wait 50 ms
    int ch;

    int row = 0;
    mvprintw(row++, 0, "Publishing to %s", output_topic.c_str());
    mvprintw(row++, 0, "Use w, a, s, d to drive, space to halt, and q to quit");
    while(node->ok()) {
        row = 2;
        mvprintw(row++, 0, "Angular: % 0.2f", twist.angular.z);
        mvprintw(row++, 0, "Linear:  % 0.2f", twist.linear.x);
        mvprintw(row++, 0, "");
        refresh();
        ch = getch();
        if (ch == 'w') {
            twist.linear.x += v_inc;
        } else if (ch == 's') {
            twist.linear.x -= v_inc;
        } else if (ch == 'a') {
            twist.angular.z += w_inc;
        } else if (ch == 'd') {
            twist.angular.z -= w_inc;
        } else if (ch == ' ') {
            twist.angular.z = 0;
            twist.linear.x = 0;
        } else if (ch == 'q') {
            twist.angular.z = 0;
            twist.linear.x = 0;
            pub.publish(twist);
            ros::Duration(0.5).sleep();
            break;
        } else if (ch == ERR) {
            continue;
        }
        twist.linear.x = min(max(twist.linear.x, -v_speed), v_speed);
        twist.angular.z = min(max(twist.angular.z, -w_speed), w_speed);

        pub.publish(twist);
        ros::spinOnce();
    }
    endwin();
    return 0;
}
