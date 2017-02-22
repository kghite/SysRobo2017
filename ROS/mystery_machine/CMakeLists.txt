cmake_minimum_required(VERSION 2.8.3)
project(mystery_machine)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs image_transport cv_bridge tf nav_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(forebrain src/forebrain.cpp)
target_link_libraries(forebrain ${catkin_LIBRARIES})

add_executable(midbrain src/midbrain.cpp)
target_link_libraries(midbrain ${catkin_LIBRARIES})

add_executable(arbiter src/arbiter.cpp)
target_link_libraries(arbiter ${catkin_LIBRARIES})

add_executable(sonar_tf_publisher src/sonar_tf_publisher.cpp)
target_link_libraries(sonar_tf_publisher ${catkin_LIBRARIES})

add_executable(lidar_tf_publisher src/lidar_tf_publisher.cpp)
target_link_libraries(lidar_tf_publisher ${catkin_LIBRARIES})

add_executable(sonar_transform src/sonar_transform.cpp)
target_link_libraries(sonar_transform ${catkin_LIBRARIES})