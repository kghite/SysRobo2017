cmake_minimum_required(VERSION 2.8.3)
project(mystery_machine)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs image_transport cv_bridge)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(forebrain src/forebrain.cpp)
target_link_libraries(forebrain ${catkin_LIBRARIES})

add_executable(midbrain src/midbrain.cpp)
target_link_libraries(midbrain ${catkin_LIBRARIES})

add_executable(arbiter src/arbiter.cpp)
target_link_libraries(arbiter ${catkin_LIBRARIES})