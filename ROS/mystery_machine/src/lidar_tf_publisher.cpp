#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(3.14, 0, 3.14),
            tf::Vector3(0.3302, 0.0, 0.0889)),
            ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}
