#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

long previous_left_encoder_counts = 0;
long previous_right_encoder_counts = 0;
ros::Time current_time_encoder, last_time_encoder;
double distance_per_count = (3.14159265 * 0.1524) / 168;
double lengthBetweenTwoWheels = 0.31115;

double x;
double y;
double th;

double vx;
double vy;
double vth;
double deltaLeft;
double deltaRight;

long left = 0;
long right = 0;

void updateDeltas() {
    // Break encoder message into sides
    current_time_encoder = ros::Time::now();

    deltaLeft = left - previous_left_encoder_counts;
    deltaRight = right - previous_right_encoder_counts;

    // (current_time_encoder - last_time_encoder).toSec();
    vx = deltaLeft * distance_per_count;

    // (current_time_encoder - last_time_encoder).toSec();
    vy = deltaRight * distance_per_count;

    vth = ((vy - vx)/lengthBetweenTwoWheels);

    previous_left_encoder_counts = left;
    previous_right_encoder_counts = right;
    last_time_encoder = current_time_encoder;
}

void rightCallback(const std_msgs::Int32 right_ticks) {
    ROS_INFO("Right Encoder: %i", right_ticks.data);
    right = (long) right_ticks.data;
}

void leftCallback(const std_msgs::Int32 left_ticks) {
    ROS_INFO("Left Encoder: %i", left_ticks.data);
    left = -1.0 * (long) left_ticks.data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub_left = n.subscribe("/encoder_left", 100,
        leftCallback);
    ros::Subscriber sub_right = n.subscribe("/encoder_right", 100,
        rightCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);   
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1.0);
    while(n.ok()){

        current_time = ros::Time::now();

        updateDeltas();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat =
            tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x;
        odom_tf.transform.translation.y = y;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_tf);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
        ros::spinOnce();
    }
}
