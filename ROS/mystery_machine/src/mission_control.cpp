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
#include "math.h"

#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Nav stack simple goals interface
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;

// Set up state values
enum State {
    exploring,				//
    finding_elevator,		//
    naving_to_elevator,		//
    ordering_maps,			//
    calling_elevator,		// int assigned = 1
    entering_elevator,		// int assigned = 2
    riding_elevator,		// int assigned = 3
    exiting_elevator,		// int assigned = 4
    matching_map			//
};

// This is received from the wormhole stack.
struct Floor {
    int number;
    std::string id;
};

// This is received from the wormhole stack, also.
struct FloorSet {
    std::list<Floor> floor_order;
    float certainty;
    float probs[];
};


// declaring publishers
ros::Publisher audio_pub;
ros::Publisher cmd_vel_pub;


class FSM {

    public:  
        State state;
        float elev_vel;   // velocity parameter


        // declaring odomResponse info
        float dist_traveled;
        float ang_traveled;
        geometry_msgs::Point pos_old;
        geometry_msgs::Point pos_new;
        geometry_msgs::Quaternion quat_old;
        geometry_msgs::Quaternion quat_new;

        // declaring constructor for FSM, which takes arg 'ROS node handler'
        FSM(ros::NodeHandle n);

        // declaring all msg types
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::Pose goal_pose;
        geometry_msgs::Twist cmd_vel;
        // nav_msgs::Odometry odom;
        sensor_msgs::LaserScan scan;
        std_msgs::Int8 floor;


        // declaring methods
        float explore_floor();
        geometry_msgs::Pose find_elevator(nav_msgs::OccupancyGrid search_map);
        bool nav_to_elevator(geometry_msgs::Pose elevator_pose);
        FloorSet order_maps(std::string map_store_file, 
                                std_msgs::Int8 direction);
        FloorSet elevator_interaction();
        void call_elevator();
        void enter_elevator();
        void ride_elevator();
        FloorSet exit_elevator();
        Floor match_map();

        // declaring callback methods
        void scanResponse(sensor_msgs::LaserScan scan);
        void odomResponse(nav_msgs::Odometry odom);

    private:
        // FSM::CallElevator
        int8_t curr_pace_dir = -1;
        float left_elev_scan;
        float right_elev_scan;
        uint8_t left_elev_open = 0;
        uint8_t right_elev_open = 0;

};

/*
* This is equivalnt to the init() method in python.
*/
FSM::FSM(ros::NodeHandle n) {
    elev_vel = 0.1;    // TODO: confirm this vel for elevator specifically
    //scan_changes = 0;
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
FloorSet FSM::order_maps(std::string map_store_file, 
                                std_msgs::Int8 direction){
    // Determine all possible map orders and return with a given likelihood
}

/* 
 * Provide HRI around calling elevator: Robot plays soundtrack while gently
 * rocking back and forth.
 */
void FSM::call_elevator() {

    // Publish audio state
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = 1;
    audio_pub.publish(tmp);

    std::vector<float> left_elev_scan( 
        FSM::scan.begin()+256,
        FSM::scan.begin()+511);
    std::vector<float> right_elev_scan( 
        FSM::scan.begin()+0,
        FSM::scan.begin()+255);

    if (left_elev_open) {
        // TODO: go into the left elevator
    } else if (right_elev_open) {
        // TODO: go into the right elevator
    } else {
        // TODO: read average of left half of scans
        //      read average of right half scans
        //      whichever gets bigger first decides which elevator door opens
        left_elev_scan = scan.ranges
        // Rock back and forth
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = .1 * (curr_pace_dir*=-1);
    }
}

/* 
 * Provide HRI around entering elevator.
 */
void FSM::enter_elevator() {

    ROS_INFO("FSM::enter_elevator");

    // Wait for passengers to fully exit elevator
    sleep(3);

    // Publish state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    // tmp.data = state;
    tmp.data = 2;
    audio_pub.publish(tmp);

    // Enter elevator slowly
    // TODO: how to set wpt to elevator???  @Katie
    // TODO: confirm this velocity
    cmd_vel.linear.y = FSM::elev_vel;
    cmd_vel_pub.publish(cmd_vel);

    // Once 1m from elevator’s back wall: stop and rotate 180 to face elevator doors
    if (scan.ranges[256] < 1) {
        // stop moving forward when we are < 1m from elevator's back wall
        cmd_vel.linear.y = 0;
        cmd_vel_pub.publish(cmd_vel);

        // rotate bot
        cmd_vel.angular.z = FSM::elev_vel;   // TODO: confirm this velocity
        cmd_vel_pub.publish(cmd_vel);

        // stop rotating once we've gone 180
        if (FSM::ang_traveled == M_PI or FSM::ang_traveled == -M_PI) {
         cmd_vel.angular.z = 0;
         cmd_vel_pub.publish(cmd_vel);
        }
    }

    //if (FSM::scan_changes == 0) {
        //FSM::state = riding_elevator;
    //}
}

/* 
 * Provide HRI around riding elevator.
 */
void FSM::ride_elevator() {
    // Publish state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    // tmp.data = state;
    tmp.data = 3;
    audio_pub.publish(tmp);

    //if (FSM::scan_changes == 0) {
        //FSM::state = exiting_elevator;
    //}
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
    // tmp.data = state;
    tmp.data = 4;
    audio_pub.publish(tmp);

    // Exit elevator by moving forward slowly
    // TODO: confirm this velocity
    cmd_vel.linear.y = FSM::elev_vel;
    cmd_vel_pub.publish(cmd_vel);

    // Once bot has exited elevator, stop
    float y_dist_traveled = abs(FSM::pos_new.y - FSM::pos_old.y);
    if (y_dist_traveled > 2) {
        cmd_vel.linear.y = 0;
        cmd_vel_pub.publish(cmd_vel);
    }

    // TODO: Map matching behavior goes here.

    // TODO: write trigger for next state
    // if (FSM::scan_changes == 0) {
    //     FSM::state = ride_elevator();
    // }

}


/* 
 * Runs SLAM to match a stored map to the robot location on exiting an 
 * elevator
 * 
 * return: a map ID that was matched (-1 for no match)
 */
Floor FSM::match_map() {

}

void FSM::scanResponse(const sensor_msgs::LaserScan scan) {

    // set incoming data to object's scan_new attr
    FSM::scan_new = scan.ranges;

    // TODO: 
    // First, determine if 0-256 is actually left.  Or is it right?
    // Second, make comparison more robust, i.e. not just a straight comparison between old and new (because tolerance in readings allows for slop)
    std::vector<float> scan_new_left( 
        FSM::scan_new.begin()+0,
        FSM::scan_new.begin()+256);

    std::vector<float> scan_new_right( 
        FSM::scan_new.begin()+256,
        FSM::scan_new.begin()+512);


    // compare to scan_old, setting bool scan_changes appropriately
    // If there are no changes, bool = 0.  If changes, bool = 1.
    if (scan_new_left == FSM::scan_old_left) FSM::scan_changes_left = 0;
    else FSM::scan_changes_left = 1;

    if (scan_new_right == FSM::scan_old_right) FSM::scan_changes_right = 0;
    else FSM::scan_changes_right = 1;

    // set scan_new to scan_old
    FSM::scan_old = FSM::scan_new;
    FSM::scan_old_left = scan_new_left;
    FSM::scan_old_right = scan_old_right;
}

void FSM::odomResponse(const nav_msgs::Odometry odom) {

    // set incoming data to object's new attrs
    FSM::pos_new = odom.pose.pose.position;
    FSM::quat_new = odom.pose.pose.orientation;

    // calculate distance traveled
    float x_diff = FSM::pos_new.x - FSM::pos_old.x;
    float y_diff = FSM::pos_new.y - FSM::pos_old.y;
    FSM::dist_traveled = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    
    // get yaws
    float yaw_new = (float)tf::getYaw(FSM::quat_new);
    float yaw_old = (float)tf::getYaw(FSM::quat_old);

    // force yaws to always be within range [-pi, pi]
    // see wrapPi(): 
    //    https://github.com/ManickYoj/warmup_project_2017/blob/master/scripts/utils.py
    float pi_yn = atan2(sin(yaw_new), cos(yaw_new));
    float pi_yo = atan2(sin(yaw_old), cos(yaw_old));

    // calculate angle rotated
    // see diffAngle():
    //    https://github.com/ManickYoj/warmup_project_2017/blob/master/scripts/utils.py
    float d1 = pi_yn - pi_yo;
    float d2 = 2*M_PI - abs(d1);
    if (d1 > 0) d2 = d2 * -1.0;
    if (abs(d1) < abs(d2)) FSM::ang_traveled = d1;
    else FSM::ang_traveled = d2;

    // set pose_new to old attrs
    FSM::pos_old = FSM::pos_new;
    FSM::quat_old = FSM::quat_new;

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "mission_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ROS_INFO("main");

    // Init mission controller to first state on ground floor
    FSM mission_controller(n);   // init FSM
    mission_controller.state = calling_elevator;
    mission_controller.floor.data = 1;

    // Publishers
    audio_pub = n.advertise<std_msgs::Int8>("/audio_cmd", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Subscribers
    ros::Subscriber sub_scan = n.subscribe("/scan", 1000, &FSM::scanResponse,
            &mission_controller);
    ros::Subscriber sub_odom = n.subscribe("/odom", 1000, &FSM::odomResponse,
            &mission_controller);

    MoveBaseClient ac("move_base", true);

    // Move base server
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (ros::ok()) {

        if (mission_controller.state == calling_elevator) {
        	mission_controller.call_elevator();
        }

        // // switch based on FSM case
        // switch(mission_controller.state) {
        //     // case exploring:
        //     //     mission_controller.explore_floor();
        //     //     break;

        //     // case finding_elevator:
        //     //     mission_controller.find_elevator();
        //     //     break;

        //     // case naving_to_elevator:
        //     //     mission_controller.nav_to_elevator();
        //     //     break;

        //     // case ordering_maps:
        //     //     mission_controller.order_maps();
        //     //     break;

        //     case calling_elevator:
        //         mission_controller.call_elevator();
        //         break;

        //     case entering_elevator:
        //         mission_controller.enter_elevator();
        //         break;

        //     case riding_elevator:
        //         mission_controller.ride_elevator();
        //         break;

        //     case exiting_elevator:
        //         mission_controller.exit_elevator();
        //         break;

        //     // case matching_map:
        //     //     mission_controller.match_map();
        //     //     break;

        //     default:
        //         break;
        // }

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Move succeeded");
        }
        else {
            ROS_INFO("Move failed");
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
