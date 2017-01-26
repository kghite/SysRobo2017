// Arbiter
// Take Twist messages from wpt/cmd_vel and obst/cmd_vel
// Publish cmd_vel for rosserial to Arduino

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // cmd_vel data


// Inputs
geometry_msgs::Twist ang;

// Outputs
ros::Publisher pub_arb;
geometry_msgs::Twist msg;


// Subscriber callbacks
void getAngular(const geometry_msgs::Twist ang_vel) {
	// Set global reference to angular message input
	ang = ang_vel;
}

void getLinear(const geometry_msgs::Twist lin_vel) {
	// Compute combination of angular and linear inputs
	msg.angular = ang.angular;
	msg.linear = lin_vel.linear;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbiter");
  ros::NodeHandle n;

  // Subscribe to compass and gps data
  ros::Subscriber sub_ang = n.subscribe("ang/cmd_vel", 1000, getAngular);
  ros::Subscriber sub_lin = n.subscribe("lin/cmd_vel", 1000, getLinear);

  // Publish final command velocity
  pub_arb = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  pub_arb.publish(msg);

  ros::spin();

  return 0;
}