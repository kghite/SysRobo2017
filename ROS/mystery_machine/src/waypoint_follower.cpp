// Waypoint Follower
// Take in IMU magnetometer data on /imu/mag and GPS in /fix
// Read waypoints from text file
// Output theta value to /goal/theta

#include <iostream>
#include <fstream>
#include <string> // Read waypoint file
#include <stdlib.h> // Convert strings to floats
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h" // GPS data
#include "std_msgs/Float64.h" // Theta output
#include "geometry_msgs/Vector3Stamped.h" // IMU compass data

// ROS Input: gps, compass
sensor_msgs::NavSatFix gps_pos;
geometry_msgs::Vector3Stamped imu_mag;

// File Input: waypoints
float waypoints[10];

// Keep track of how many waypoints we have reached
int waypoint_status = 0;

// Output: theta
std_msgs::Float64 theta;
ros::Publisher pub_theta;

// GPS Callback
void getGPS(const sensor_msgs::NavSatFix gps) 
{
  // Store gps data in global
  ROS_INFO("Received GPS Data");
  gps_pos = gps;
}

// Read in a text file with race 4 waypoints in order
void readGoals()
{
  std::ifstream file("race_goals.txt");
  int wp_num = 0;
  std::string str;
  while (std::getline(file, str))
  {
    // Convert string into float
    float coord = atof(str.c_str());

    // Determine if lat or long (lat = even, long = odd)
    waypoints[wp_num] = coord;

    wp_num++;
  }
}

// Compass callback: Send angular velocity to forebrain
void getCompass(const geometry_msgs::Vector3Stamped imu)
{
  ROS_INFO("Received Compass Data");
  imu_mag = imu;

  // Break down lat and long of gps position
  float robot_lat = gps_pos.latitude;
  float robot_long = gps_pos.longitude;

  // Goal GPS points
  float goal_lat = waypoints[waypoint_status];
  float goal_long = waypoints[waypoint_status + 1];

  // Check if we have reached the current goal
  if (goal_lat == robot_lat && goal_long == robot_long)
  {
    waypoint_status++;

    // Recompute new gps waypoints
    goal_lat = waypoints[waypoint_status];
    goal_long = waypoints[waypoint_status + 1];
  }

  // Break down IMU mag data
  float compass_x = imu_mag.vector.x;
  float compass_y = imu_mag.vector.y;

  // Compute robot heading (0-360) and reference East as 0
  float heading = (atan2(compass_y, compass_x) * 180) / M_PI;
  if (heading < 0)
  {
    heading = 180 + (180 - (heading * -1));
  }
  float theta_r = heading + 90;

  // Calculate angle from the waypoint goal to East
  float theta_g = atan2(robot_long - goal_long, robot_lat - robot_long) * (180/M_PI);

  // Calculate angle for robot to move
  float theta = theta_g - theta_r;

  // DEBUG
  ROS_INFO("Robot Heading: %f", theta_r);
  ROS_INFO("Heading to Goal: %f", theta);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_follower");
  ros::NodeHandle n;

  // Read in the waypoint goals in order and assign to globals
  readGoals();

  // Subscribe to compass and gps data
  ros::Subscriber sub_compass = n.subscribe("fix", 1000, getGPS);
  ros::Subscriber sub_gps = n.subscribe("imu/mag", 1000, getCompass);

  // Publish angle between robot and the goal
  pub_theta = n.advertise<std_msgs::Float64>("/goal/theta", 1000);

  pub_theta.publish(theta);

  ros::spin();

  return 0;
}
