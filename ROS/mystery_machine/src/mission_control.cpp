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
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"

// Set up state values
enum State {
    exploring,
    finding_elevator,
    naving_to_elevator,
    ordering_maps,
    calling_elevator,
    entering_elevator,
    riding_elevator,
    exiting_elevator,
    matching_map
};

// This is received from the wormhole stack.
struct Floor {
    int number;
    std::string id;
};

// This is received from the wormhole stack, also.
struct FloorSet {
    std::list<Floor> floor_order;
    float probs[];
    float certainty;
};

// fuck pointers; omitted for now


// Outputs

class FSM {

    public:  
        State state;

        // declaring information resulting from callbacks
        bool scan_changes;
        std::vector<float> scan_old;
        std::vector<float> scan_new;

        // declaring constructor for FSM, which takes arg 'ROS node handler'
        FSM(ros::NodeHandle n);

        // declaring all msg types
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::Pose goal_pose;
        geometry_msgs::Twist cmd_vel;
        nav_msgs::Odometry odom;
        sensor_msgs::LaserScan scan;
        std_msgs::Int8 floor;

        // declaring publishers
        ros::Publisher state_pub;
        ros::Publisher cmd_vel_pub;


        // declaring methods
        float explore_floor();
        geometry_msgs::Pose find_elevator(nav_msgs::OccupancyGrid search_map);
        bool nav_to_elevator(geometry_msgs::Pose elevator_pose);
        FloorSet ordering_maps(std::string map_store_file, 
                                std_msgs::Int8 direction);
        FloorSet elevator_interaction();
        void call_elevator();
        void enter_elevator();
        void ride_elevator();
        FloorSet exit_elevator();
        Floor map_matching();

        // declaring callback methods
        void scanResponse(sensor_msgs::LaserScan scan);
};

/*
* This is equivalnt to the init() method in python.
*/
FSM::FSM(ros::NodeHandle n) {

    // initializing
    state = exploring;
    cmd_vel = geometry_msgs::Twist();
    scan_changes = 0;   // 0 = no change; 1 = changed
    // scan_old[512] = 0;
    // scan_new[512] = 0;

    // declaring & initializing publishers
    state_pub = ros::Publisher(n.advertise<std_msgs::Int8>("/bot_state", 1000));
    cmd_vel_pub = ros::Publisher(n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000));

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
    // Note that wpt must be positioned such that scan can detect both elevators
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
 * Provide HRI around calling elevator: Robot plays soundtrack while gently
 * rocking back and forth.
 */
void FSM::call_elevator() {

    // Publish state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = state;
    state_pub.publish(tmp);

    // Rock back and forth

}

/* 
 * Provide HRI around entering elevator.
 */
void FSM::enter_elevator() {

    // Wait for passengers to fully exit elevator
    sleep(3);

    // Publish state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = state;
    state_pub.publish(tmp);

    // Enter elevator slowly (assume right-angle navigation)
    // TODO: confirm this velocity
    cmd_vel.linear.y = 0.001;
    cmd_vel_pub.publish(cmd_vel);

    // Once 1m from elevator’s back wall: stop and rotate 180 to face elevator doors
    if (scan.ranges[256] < 1) {
        // stop moving forward when we are < 1m from elevator's back wall
        cmd_vel.linear.y = 0;
        cmd_vel_pub.publish(cmd_vel);

        // rotate 180 deg
        cmd_vel.angular.z = 0.01;
        cmd_vel_pub.publish(cmd_vel);

        // TODO: format odom correctly to track angular change
        // // stop rotating once we've gone 180
        // if (odom == 180) {
        //  cmd_vel.angular.z = 0;
        //  cmd_vel_pub.publish(cmd_vel);
        // }
    }
}

/* 
 * Provide HRI around riding elevator.
 */
void FSM::ride_elevator() {
    // Publish state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = state;
    state_pub.publish(tmp);
}

 /*
 * Provide HRI around exiting elevator.
 *
 * return: the list of possible floor IDs that the robot could be at on 
 * elevator exit
 */
FloorSet FSM::exit_elevator() {

    // Publish state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = state;
    state_pub.publish(tmp);

    // Exit elevator slowly
    // TODO: confirm this velocity
    cmd_vel.linear.y = 0.05;
    cmd_vel_pub.publish(cmd_vel);

    // TODO: format odom correctly to track y dist traveled
    // // Once bot has exited elevator, stop
    // if (odom < 2) {
    //     cmd_vel.linear.y = 0;
    //     cmd_vel_pub.publish(cmd_vel);
    // }

    // Map matching behavior goes here.

}


/* 
 * Runs SLAM to match a stored map to the robot location on exiting an 
 * elevator
 * 
 * return: a map ID that was matched (-1 for no match)
 */
Floor FSM::map_matching() {

}

void FSM::scanResponse(const sensor_msgs::LaserScan scan) {

    // TODO: don't forget to initialize scan_new, scan_old, and scan_bool in "init" method above

    // set incoming data to scan_new
    FSM::scan_new = scan.ranges;

    // compare to scan_old, setting bool scan_changes appropriately
    if (FSM::scan_new == FSM::scan_old) {
        FSM::scan_changes = 1;
    } else {
        FSM::scan_changes = 0;
    }

    // set scan_new to scan_old
    FSM::scan_old = FSM::scan_new;
}

void odomResponse(const nav_msgs::Odometry odom) {

}


void stateResponse(const std_msgs::Int16 estop) {
    // This callback actually runs everything
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle n;

    // Init mission controller to first state on ground floor
    FSM mission_controller(n);   // init FSM
    mission_controller.state = exploring;
    mission_controller.floor.data = 1;

    // Continuously check the state and run the appropriate class methods
    // States get changed in the methods only
    // Do all this in a callback so will pause if the e-stop is pressed

    ros::Subscriber sub_estop = n.subscribe("/estop", 200, stateResponse);

    // TODO: confirm rate for the following subscribers
    ros::Subscriber sub_scan = n.subscribe("/scan", 200, &FSM::scanResponse, &mission_controller);
    ros::Subscriber sub_odom = n.subscribe("/odom", 200, odomResponse);

    ros::spin();

    return 0;
}