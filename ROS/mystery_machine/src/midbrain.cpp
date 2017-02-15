// Midbrain controller
// Take in LIDAR data on /scan
// Output Int16MultArray on /obst/cmd_vel to turn toward waypoint set on /goal

#include <string>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// Input
sensor_msgs::LaserScan scan;

// Output
ros::Publisher *pub_arb;
geometry_msgs::Twist cmd_vel;

// Linear velocity delta
int lin_vel = 0;

// Speed limit
float limit = 0.7;

void controlSpeed(const sensor_msgs::LaserScan lidar_scan)
{
  ///// FILTER LIDAR SCANS /////
  // Assign LIDAR scan to global
  scan = lidar_scan;

  long number_of_ranges = lidar_scan.ranges.size();
  float forward_distance = 100;

  // Remove junk values from scan data (0.0 is out of range or no read)
  for(int i=0; i < sizeof(scan.ranges) / sizeof(scan.ranges[0]); i++)
  {
    if(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
    {
      scan.ranges[i] = 0.0;
    }
  }

  // Limit scan to range
  for (long i = number_of_ranges/3; i<2*number_of_ranges/3;i++)
  {
      if (forward_distance > scan.ranges[i] && scan.ranges[i] != 0)
          forward_distance = scan.ranges[i];
  }

  ///// CALCULATE VELOCITY /////
  ROS_INFO("Forward distance: %lf", forward_distance);
  if (forward_distance < .3)
  {
    // Move backward
    ROS_INFO("Backward");
    lin_vel = -0.1;
  }
  else if (forward_distance < .6)
  {
    // Stop
    ROS_INFO("Stop");
    lin_vel = 0;
  }
  else
  {
    // Move forward
    ROS_INFO("Forward");
    lin_vel = 0.1;
  }

  // Linear vel
  float linear_changed = cmd_vel.linear.y + lin_vel;

  // Enforce speed limitation
  if (linear_changed > limit) {
    cmd_vel.linear.y = limit;
  }
  // Increment or decrement cmd_vel
  else {
    cmd_vel.linear.y = linear_changed;
  }

  pub_arb->publish(cmd_vel);

  // DEBUG
  ROS_INFO("Publishing Output");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "midbrain");

  ros::NodeHandle n;

  ros::Subscriber sub_imu = n.subscribe("/scan", 1000, controlSpeed);

  pub_arb = new ros::Publisher(n.advertise<geometry_msgs::Twist>("obst/cmd_vel", 1000));

  ros::spin();

  return 0;
}
