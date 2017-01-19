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

add_executable(cone_detection src/cone_detection.cpp)
target_link_libraries(cone_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(waypoint_follower src/waypoint_follower.cpp)
target_link_libraries(waypoint_follower ${catkin_LIBRARIES})