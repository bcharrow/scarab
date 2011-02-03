#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

ros::Publisher pubarr;
visualization_msgs::MarkerArray markers;
visualization_msgs::Marker pose;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    pose.header.frame_id = msg.header.frame_id;
    pose.header.stamp = ros::Time::now();
    pose.action = visualization_msgs::Marker::ADD;
    pose.lifetime = ros::Duration();

    pose.ns = "pamcl_markers";
    pose.id = 0;

    pose.type = visualization_msgs::Marker::CYLINDER;

    double lambda1, lambda2, common;
    double a, b, c, d;
    a = msg.pose.covariance[0];
    b = msg.pose.covariance[1];
    c = msg.pose.covariance[6];
    d = msg.pose.covariance[7];
    if (b != c) {
        ROS_ERROR("crap");
    }
    common = sqrt((a-d)*(a-d) + 4 * c * c) / 2;
    lambda1 = (a + d) / 2 - common;
    lambda2 = (a + d) / 2 + common;

    pose.color.r = 0.0f;
    pose.color.g = 1.0f;
    pose.color.b = 0.0f;
    pose.color.a = 0.8f;

    pose.scale.x = 0.2f;
    pose.scale.y = 0.2f;
    pose.scale.z = 0.2f;
    // pose.scale.x = lambda1;
    // pose.scale.y = lambda2;
    // pose.scale.z = 0.2f;

    pose.pose.position = msg.pose.pose.position;

    markers.markers.clear();
    markers.markers.push_back(pose);
    pubarr.publish(markers);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pamcl_marker");

    ros::NodeHandle n("~");
    std::string fname;

    ros::Subscriber sub = n.subscribe("pose", 1, pose_callback);

    // Advertise both channels so rviz can find it easily
    ros::Publisher pub_unused = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    pubarr = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);


    ros::spin();
}
