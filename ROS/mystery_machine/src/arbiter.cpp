// Arbiter
// Take Twist messages from wpt/cmd_vel and obst/cmd_vel
// Publish cmd_vel for rosserial to Arduino

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // cmd_vel data


// Inputs
geometry_msgs::Twist wpt;

// Outputs
ros::Publisher pub_cmd_vel;

// Subscriber callbacks
void getAngular(const geometry_msgs::Twist ang_vel) {
	// Set global reference to angular message input
	wpt = ang_vel;
}

void getLinear(const geometry_msgs::Twist lin_vel) {
	// Compute combination of angular and linear inputs
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbiter");
  ros::NodeHandle n;

  // Subscribe to compass and gps data
  ros::Subscriber sub_compass = n.subscribe("wpt/cmd_vel", 1000, getAngular);
  ros::Subscriber sub_gps = n.subscribe("obst/cmd_vel", 1000, getLinear);

  // Publish final command velocity
  pub_theta = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  pub_theta.publish(theta);

  ros::spin();

  return 0;
}