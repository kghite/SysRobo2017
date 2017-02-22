#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::PointStamped sonar_point;
geometry_msgs::PointStamped sonar_transformed;
ros::Publisher pub;

// Subscriber callback for sonar
void getSonar(const geometry_msgs::PointStamped sonar) {
  // Set global reference to angular message input
  sonar_point = sonar;
}

void transformPoint(const tf::TransformListener& listener){
  sonar_point.header.frame_id = "base_sonar";

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", sonar_point, base_point);

    ROS_INFO("base_sonar: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        sonar_point.point.x, sonar_point.point.y, sonar_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_sonar\" to \"base_link\": %s", ex.what());
  }

  sonar_transformed = sonar_point;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "sonar_transform");
  ros::NodeHandle n;

  ros::Subscriber sub_ang = n.subscribe("ang/cmd_vel", 500, getSonar);

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(0.5), boost::bind(&transformPoint, boost::ref(listener)));

  pub = n.advertise<geometry_msgs::PointStamped>("/sonar_transformed", 500);

  pub.publish(sonar_transformed);

  ros::spin();

}