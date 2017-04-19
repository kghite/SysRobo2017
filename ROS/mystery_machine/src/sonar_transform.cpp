/*
 * Subscribe to the \sonar_data Point message, turn it into a PointStamped
 * message attached to the base_sonar tf, and publish it as \sonar_transformed
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher pub;

void getSonar(geometry_msgs::PointStamped msg) {
    // Attach to the base_sonar tf
    msg.header.frame_id = "base_sonar";
    msg.header.stamp = ros::Time();

    // Convert from cm to m
    msg.point.x = msg.point.x / 100;
    msg.point.y = msg.point.y / 100;
    msg.point.z = msg.point.z / 100;

    // Publish the new PointStamped message
    pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_transform");
    ros::NodeHandle n;

    ros::Subscriber sub_ang = n.subscribe("sonar_data", 50, getSonar);

    pub = n.advertise<geometry_msgs::PointStamped>("sonar_transformed", 50);

    ros::spin();
}

