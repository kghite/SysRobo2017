/*
 * Subscribe to the \sonar_data SonarScan message, turn it into a LaserScan
 * message attached to the base_sonar tf, and publish it as \sonar_transformed
 */


#include <ros/ros.h>
#include <mystery_machine/SonarScan.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>


#define PI 3.14159265358979323846


ros::Publisher pub;


/*
 * Callback function for /sonar_data topic. Creates a LaserScan message from a
 * SonarScan message. The new LaserScan message can be used to create maps.
 *
 * msg: SonarScan message from subscriber
 */
void sonar_callback(mystery_machine::SonarScan msg) {

    // Define LaserScan ranges[] attribute from SonarScan message
    std::vector <float> ranges;
    float curr_range;
    for (int i=0; i<180; i++) {
        curr_range = (i==msg.angle) ? msg.range : 0;
        ranges.push_back(curr_range);
    }

    // Create the new LaserScan message from the SonarScan message
    sensor_msgs::LaserScan sonar_laser_scan;
    sonar_laser_scan.header = msg.header;
    sonar_laser_scan.header.stamp = ros::Time::now();
    sonar_laser_scan.angle_min = 0;
    sonar_laser_scan.angle_max = PI;
    sonar_laser_scan.angle_increment = PI/179.0;
    sonar_laser_scan.time_increment = .05/180.0;
    sonar_laser_scan.scan_time = .05;
    sonar_laser_scan.range_min = msg.range_min;
    sonar_laser_scan.range_max = msg.range_max;
    sonar_laser_scan.ranges = ranges;

    // Publish the new PointStamped message
    pub.publish(sonar_laser_scan);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "sonar_transform");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("sonar_data", 50, sonar_callback);

    pub = n.advertise<sensor_msgs::LaserScan>("sonar_scan", 10);

    ros::spin();
}

