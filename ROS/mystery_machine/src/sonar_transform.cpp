/*
 * Subscribe to the /sonar_data SonarScan message, insert the data from
 * /sonar_data into /scan to create a merged LaserScan message of the lidar
 * scan and the sonar scan called /merged_scan.
 */


#include <ros/ros.h>
#include <mystery_machine/SonarScan.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>


#define PI 3.14159265358979323846

mystery_machine::SonarScan sonar_scan;
ros::Publisher merged_pub;


/*
 * Callback function for /sonar_data topic. Stores the SonarScan as a global.
 *
 * msg: SonarScan message from subscriber
 */
void sonar_callback(mystery_machine::SonarScan msg) {

    // Store the SonarScan message as a global
    sonar_scan = msg;
}


/*
 * Callback function for /scan topic. Creates a LaserScan message by joining
 * /scan and /sonar_scan. Published to /merged_scan. The new LaserScan message
 * can be used to create maps.
 *
 * msg: LaserScan message from subscriber
 */
void lidar_callback(sensor_msgs::LaserScan msg) {

    // Define the viable range of the sonar (in meters)
    float sonar_min_range = 0.0;
    float sonar_max_range = 2.0;

    // If the sonar reading is within the viable range of the sonar
    if (sonar_min_range <= sonar_scan.range &&
            sonar_scan.range <= sonar_max_range) {

        // Insert the sonar scan data into the lidar scan
        // 512 elements in lidar scan, 180 elements in sonar scan
        int insert_index = 512 - round(sonar_scan.angle / 180.0 * 512.0);
        msg.ranges[insert_index] = sonar_scan.range;
    }

    // Publish the new merged LaserScan message
    merged_pub.publish(msg);
}


int main(int argc, char** argv) {

    // Initialize ros node
    ros::init(argc, argv, "sonar_transform");
    ros::NodeHandle n;

    // Initialize subscribers to sonar and lidar data
    ros::Subscriber sonar_sub = n.subscribe("sonar_data", 50, sonar_callback);
    ros::Subscriber lidar_sub = n.subscribe("scan", 50, lidar_callback);

    // Initialize publisher for merged scan topic
    merged_pub = n.advertise<sensor_msgs::LaserScan>("merged_scan", 10);

    ros::spin();
}

