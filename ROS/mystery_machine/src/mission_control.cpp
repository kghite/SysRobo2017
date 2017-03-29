#include "string"
#include "list"
#include "iostream"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8.h"

// Set up state values
enum State {
	exploring,
	finding_elevator,
	naving_to_elevator,
	ordering_maps,
	riding_elevator,
	matching_map
};

class FSM {
	public:
		geometry_msgs::PoseStamped current_pose;
		geometry_msgs::goal_pose;
		Int8 floor;

		int explore_floor();
		geometry_msgs::Pose find_elevator(nav_msgs::OccupancyGrid search_map);
		bool nav_to_elevator(geometry_msgs::Pose elevator_pose);
		list<string> ordering_maps(string map_store_file);
		int elevator_interaction();
		int map_matching();
}

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
}

/* 
 * Determines the probabilities of floor orders for the current map set
 * 
 * Input: a string file name for the map_stor YAML file, a std_msgs Int8
 * indicating the last direction of the robot in the elevator (up or down)
 *
 * return: a list of map IDs in orders with order probabilities
 */
int[][] FSM::ordering_maps(string map_store_file, 
								std_msgs::Int8 direction){
	// Determine all possible map orders and return with a given likelihood
}

/* 
 * Provide HRI and elevator behaviors such as loading, riding, and unloading
 *
 * return: the list of possible floor IDs that the robot could be at on 
 * elevator exit
 */
int[] FSM::elevator_interaction() {
	// Do normal interaction things
}

/* 
 * Runs SLAM to match a stored map to the robot location on exiting an 
 * elevator
 * 
 *
 * return: a map ID that was matched (-1 for no match)
 */
int FSM::map_matching() {

}

int main() {
	// Init mission controller to first state
	FSM mission_controller;
	mission_controller.state = explore_floor;

	// Continuously check the state and run the appropriate class methods
	// States get changed in the methods only
}