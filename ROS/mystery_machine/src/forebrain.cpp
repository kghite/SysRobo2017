// Forebrain controller
// Take in LIDAR and camera topics 
// Output Int8MultArray on /wpt/cmd_vel to turn away from walls and cones

#include <string>
#include <vector>
#include <numeric>
#include "ros/ros.h"
#include "geometry_msgs/Twist"
#include "sensor_msgs/LaserScan.h"

// Inputs
geometry_msgs::Twist cmd_vel;
sensor_msgs::LaserScan filtered_scan;
sensor_msgs::LaserScan scan;

// Outputs
ros::Publisher pub_arb;

void getLIDAR(const sensor_msgs::LaserScan lidar_scan) {
  // Set cmd_vel to publish to the arbiter
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forebrain");
  ros::NodeHandle n;

  ros::Subscriber sub_lidar = n.subscribe("scan", 1000, getLIDAR);

  pub_arb = n.advertise<geometry_msgs::Twist>("wpt/cmd_vel", 1000);

  pub_arb.publish(cmd_array);

  ros::spin();

  return 0;
}
