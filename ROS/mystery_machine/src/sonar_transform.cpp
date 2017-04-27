/*
 * Subscribe to the /sonar_data SonarScan message, turn it into a LaserScan
 * message attached to the base_sonar tf, and publish it as /sonar_scan.
 * Insert the data from /sonar_data into /scan to create a merged LaserScan
 * message of the lidar scan and the sonar scan called /merged_scan.
 */


#include <ros/ros.h>
#include <mystery_machine/SonarScan.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>


#define PI 3.14159265358979323846

mystery_machine::SonarScan sonar_scan;
ros::Publisher sonar_pub;
ros::Publisher merged_pub;


/*
 * Callback function for /sonar_data topic. Creates a LaserScan message from a
 * SonarScan message. The new LaserScan message can be used to create maps.
 *
 * msg: SonarScan message from subscriber
 */
void sonar_callback(mystery_machine::SonarScan msg) {

    // Store the SonarScan message as a global
    sonar_scan = msg;

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

    // Publish the new LaserScan message
    sonar_pub.publish(sonar_laser_scan);
}


/*
 * Callback function for /scan topic. Creates a LaserScan message by joining
 * /scan and /sonar_scan. Published to /merged_scan. The new LaserScan message
 * can be used to create maps.
 *
 * msg: LaserScan message from subscriber
 */
void lidar_callback(sensor_msgs::LaserScan msg) {

    // Insert the sonar scan data into the lidar scan
    // 512 elements in lidar scan, 180 elements in sonar scan
    int insert_index = 512 - round(sonar_scan.angle / 180.0 * 512.0);
    msg.ranges[insert_index] = sonar_scan.range;

    // Publish the new PointStamped message
    merged_pub.publish(msg);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "sonar_transform");
    ros::NodeHandle n;

    ros::Subscriber sonar_sub = n.subscribe("sonar_data", 50, sonar_callback);
    ros::Subscriber lidar_sub = n.subscribe("scan", 50, lidar_callback);

    sonar_pub = n.advertise<sensor_msgs::LaserScan>("sonar_scan", 10);
    merged_pub = n.advertise<sensor_msgs::LaserScan>("merged_scan", 10);

    ros::spin();
}

