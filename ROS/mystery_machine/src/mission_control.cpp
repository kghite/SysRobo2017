/*
 * File: mission_control.cpp
 * 
 * Updated: 3/29/17
 * 
 * Description: ROS integrated FSM for building mapping and elevator
 * exploration with the Mystery Machine  
*/

#include "string"
#include "list"
#include "iostream"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"

// Set up state values
enum State {
    exploring,
    finding_elevator,
    naving_to_elevator,
    ordering_maps,
    riding_elevator,
    matching_map
};

struct Floor {
    int number;
    std::string id;
};

struct FloorSet {
    std::list<Floor> floor_order;
    float probs[];
    float certainty;
};

class FSM {
    public:
        State state;
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::Pose goal_pose;
        std_msgs::Int8 floor;

        float explore_floor();
        geometry_msgs::Pose find_elevator(nav_msgs::OccupancyGrid search_map);
        bool nav_to_elevator(geometry_msgs::Pose elevator_pose);
        FloorSet ordering_maps(std::string map_store_file, 
                                std_msgs::Int8 direction);
        FloorSet elevator_interaction();
        Floor map_matching();
};

/* 
 * Runs gmapping to explore an unmapped floor to some percentage complete
 *
 * return: an integer representing the estimated completeness percentage
 * of the map
 */
float FSM::explore_floor() {
    // Gmapping integration
}

/* 
 * Finds an elevator in a given occupancy grid using an assumed shape
 * 
 * Input: a nav_msgs/OccupancyGrid representing the current map to search 
 * for an elevator within
 *
 * return: a robot pose at the elevator doors for loading
 */
geometry_msgs::Pose FSM::find_elevator(nav_msgs::OccupancyGrid search_map) {
    // Find an elevator within a map
}

/* 
 * Navigate to an elevator given the given elevator pose
 *
 * Input: a geometry_msgs/Pose for the elevator loading position
 *
 * return: a boolean indicator that the robot has reached the target pose
 */
bool FSM::nav_to_elevator(geometry_msgs::Pose elevator_pose) {
    // Go to an elevator loading position
    // Note that the wpt must be positioned such that the lidar can detect both elevators
}

/* 
 * Determines the probabilities of floor orders for the current map set
 * 
 * Input: a string file name for the map_stor YAML file, a std_msgs Int8
 * indicating the last direction of the robot in the elevator (up or down)
 *
 * return: a list of map IDs in orders with order probabilities
 */
FloorSet FSM::ordering_maps(std::string map_store_file, 
                                std_msgs::Int8 direction){
    // Determine all possible map orders and return with a given likelihood
}

/* 
 * Provide HRI and elevator behaviors such as loading, riding, and unloading
 *
 * return: the list of possible floor IDs that the robot could be at on 
 * elevator exit
 */
FloorSet FSM::elevator_interaction() {
    // Do normal interaction things
}

/* 
 * Runs SLAM to match a stored map to the robot location on exiting an 
 * elevator
 * 
 *
 * return: a map ID that was matched (-1 for no match)
 */
Floor FSM::map_matching() {

}

void stateResponse(const std_msgs::Int16 estop) {
    // This callback actually runs everything
}

int main(int argc, char **argv) {
    // Init mission controller to first state on ground floor
    FSM mission_controller;
    mission_controller.state = exploring;
    mission_controller.floor.data = 1;

    ros::init(argc, argv, "mission_control");

    ros::NodeHandle n;

    // Continuously check the state and run the appropriate class methods
    // States get changed in the methods only
    // Do all this in a callback so will pause if the e-stop is pressed
    ros::Subscriber sub_estop = n.subscribe("/estop", 200, stateResponse);

    ros::spin();

    return 0;
}