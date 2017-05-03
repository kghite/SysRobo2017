/*
 * File: mission_control.cpp
 * 
 * Updated: 3/29/17
 * 
 * Description: ROS integrated FSM for building mapping and elevator
 * exploration with the Mystery Machine  
*/


#include <string>
#include <list>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <tf/transform_datatypes.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "utils.h"


// Nav stack simple goals interface
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;

// Set up m_state values
enum State {
    booting,
    exploring,
    finding_elevator,
    naving_to_elevator,
    ordering_maps,
    calling_elevator,
    entering_elevator,
    riding_elevator,
    exiting_elevator,
    matching_map,
    stopping,
    turning
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
ros::Publisher audio_cmd_pub;
ros::Publisher cmd_vel_pub;


class FSM {

    public:  
        State m_state;
        State m_prev_state;
        float m_elev_vel;   // velocity parameter
        int m_motion_status; // Motion sequence control

        std::vector <float> m_scan;

        // Declaring odom_callback info
        float m_dist_traveled;
        float m_ang_traveled;
        geometry_msgs::Point m_pos_old;
        geometry_msgs::Point m_curr_pos;
        geometry_msgs::Quaternion m_quat_old;
        geometry_msgs::Quaternion m_curr_quat;
        float m_curr_yaw;
        float m_goal_yaw;
        float m_yaw_thresh;

        // Declaring constructor for FSM, which takes arg 'ROS node handler'
        FSM(ros::NodeHandle n);

        // Declaring all msg types
        geometry_msgs::PoseStamped m_current_pose;
        geometry_msgs::Pose m_goal_pose;
        geometry_msgs::Twist m_cmd_vel;
        //nav_msgs::Odometry m_odom;
        std_msgs::Int8 m_floor;

        // Declare methods
        float explore_floor();
        geometry_msgs::Pose find_elevator(nav_msgs::OccupancyGrid search_map);
        bool nav_to_elevator(geometry_msgs::Pose elevator_pose);
        FloorSet order_maps(std::string map_store_file, 
                std_msgs::Int8 direction);
        FloorSet elevator_interaction();
        void boot();
        void call_elevator();
        void enter_elevator();
        void ride_elevator();
        void stop();
        void turn_pid();
        FloorSet exit_elevator();
        Floor match_map();

        // declaring callback methods
        void scan_callback(sensor_msgs::LaserScan scan);
        void odom_callback(nav_msgs::Odometry odom);

    private:
        // FSM::boot
        int m_loops_to_spend_in_boot = 20;
        int m_loops_spent_in_boot = 0;

        // FSM::call_elevator
        float m_closed_left_elev_scan_avg = 0.0;
        float m_closed_right_elev_scan_avg = 0.0;
        float m_elev_open_thresh = 0.2;
        uint8_t m_left_elev_open = 0;
        uint8_t m_right_elev_open = 0;
        int8_t m_curr_pace_dir = -1;

        // FSM::enter_elevator
        float m_kp = 0.1;
        float m_ki = 0.1;
        float m_kd = 0.1;
        float m_curr_error = 0.0;
        float m_prev_error = 0.0;
        float m_sum_error = 0.0;
        float m_diff_error = 0.0;
        float m_ang_vel = 0.0;
        float m_elevator_yaw = 0.78;
};


/*
* This is equivalnt to the init() method in python.
*/
FSM::FSM(ros::NodeHandle n) {
    m_elev_vel = 0.09; // speed for moving around the elevator
    m_scan = std::vector <float> (512, 0.0); // fill scan with zeros
    m_motion_status = 0; // Init motion sequence counter

    // PID
    m_yaw_thresh = 0.05; // in radians
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


void FSM::boot() {

    ROS_INFO("Booting");

    // Make sure we don't play sound from a previous session
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = 4;
    audio_cmd_pub.publish(tmp);

    if (m_loops_to_spend_in_boot - m_loops_spent_in_boot <= 0) {
        // Test PID control
        // m_goal_yaw = m_curr_yaw - 1.57;
        // m_prev_state = stopping;
        // m_state = turning;

        // Start at elevator interaction
        m_state = calling_elevator;
    }
    else {

        m_loops_spent_in_boot++;
    }
}


/* 
 * Provide HRI around calling elevator: Robot plays soundtrack while waiting
 * for one of the elevator doors to open
 */
void FSM::call_elevator() {

    ROS_INFO("Calling elevator");

    // Publish appropriate audio m_state
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = 1;
    audio_cmd_pub.publish(tmp);

    // Split current LaserScan into left and right sides
    std::vector<float> left_elev_scan(m_scan.begin()+256,
            m_scan.begin()+511);
    std::vector<float> right_elev_scan(m_scan.begin()+0,
            m_scan.begin()+255);

    // Find average distance on left and right sides of LaserScan
    float avg_left_elev_scan = avg(left_elev_scan);
    float avg_right_elev_scan = avg(right_elev_scan);

    // Debug print statements
    ROS_INFO("Closed elev scan avg: %f", m_closed_right_elev_scan_avg);
    ROS_INFO("Avg right elevator scan: %f", avg_left_elev_scan);

    // If a baseline for what the LaserScan looks like when the elevators are
    // closed has not been defined, then define it
    if (m_closed_left_elev_scan_avg == 0.0 &&
            m_closed_right_elev_scan_avg == 0.0) {

        m_closed_left_elev_scan_avg = avg_left_elev_scan;
        m_closed_right_elev_scan_avg = avg_right_elev_scan;
    }

    // If the left elevator door opened
    if (avg_left_elev_scan - m_closed_left_elev_scan_avg >=
            m_elev_open_thresh) {

        ROS_INFO("Left elevator opened");
        m_left_elev_open = 1;
        m_state = entering_elevator;
    }

    // If the right elevator door opened
    else if (avg_right_elev_scan - m_closed_right_elev_scan_avg >=
            m_elev_open_thresh) {

        ROS_INFO("Right elevator opened");
        m_right_elev_open = 1;
        m_state = entering_elevator;
    }
}


/* 
 * Provide HRI around entering elevator.
 */
void FSM::enter_elevator() {
    // Publish m_state to allow Sound Arduino to do its thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = 2;
    audio_cmd_pub.publish(tmp);

    switch(m_motion_status) {
        case 0:
            ROS_INFO("Turn to elevator");
            // Turn toward the open door
            if (m_right_elev_open) {
                // Set the door back to closed for riding
                m_goal_yaw = m_curr_yaw + m_elevator_yaw;
                m_prev_state = m_state;
                m_state = turning;
            }
            else if (m_left_elev_open) {
                m_goal_yaw = m_curr_yaw - m_elevator_yaw;
                m_prev_state = m_state;
                m_state = turning;
            }
            break;
        case 1:
            ROS_INFO("Drive forward");
            m_cmd_vel.linear.x = 0.1;
            cmd_vel_pub.publish(m_cmd_vel);

            ros::Duration(1.0).sleep();

            m_cmd_vel.linear.x = 0.0;
            cmd_vel_pub.publish(m_cmd_vel);
            m_motion_status++;
            break;
        case 2: 
            ROS_INFO("Turn sqaure");
            // Turn back sqaure
            if (m_right_elev_open) {
                // Set the door back to closed for riding
                m_right_elev_open = 0;
                m_goal_yaw = m_curr_yaw - m_elevator_yaw;
                m_prev_state = m_state;
                m_state = turning;
            }
            else if (m_left_elev_open) {
                m_left_elev_open = 0;
                m_goal_yaw = m_curr_yaw + m_elevator_yaw;
                m_prev_state = m_state;
                m_state = turning;
            }
            break;
        case 3: 
            ROS_INFO("Drive into elevator");
            // Stop once set dist from elevator’s back wall
            ROS_INFO("Forward scan: %f", m_scan.at(256));
            //if (m_scan.at(256) < 1) {
            // stop moving forward when we are < 1m from elevator's back wall
            m_motion_status++;
            break;

        case 4:
            ROS_INFO("Turn around");
            // rotate bot 180 to face the door
            m_goal_yaw = m_curr_yaw + 3.14;
            m_prev_state = m_state;
            m_state = turning;
            break;
        case 5:
            ROS_INFO("Finished entering elevator");
            m_motion_status = 0;
            m_state = riding_elevator;
            break;
        default:
            break;
    }
}


/* 
 * Provide HRI around riding elevator.
 */
void FSM::ride_elevator() {
    // Publish m_state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    // tmp.data = m_state;
    tmp.data = 0;
    audio_cmd_pub.publish(tmp);

    // Check for the doors opening
}

/*
 * Stop the robot.
 */
void FSM::stop() {
    ROS_INFO("Stopping.");

    m_cmd_vel.linear.x = 0.0;
    m_cmd_vel.angular.z = 0.0;
    cmd_vel_pub.publish(m_cmd_vel);
}


/*
 * Turn the robot to m_goal_yaw
 * Return to the previous state when finished
 */
void FSM::turn_pid() {

    ROS_INFO("Turning");

    // Publish m_state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = 1;
    audio_cmd_pub.publish(tmp);

    // Use PID control to turn until close enough to the turn goal
    if (abs(m_goal_yaw - m_curr_yaw) > m_yaw_thresh) {

        m_curr_error = m_goal_yaw - m_curr_yaw;
        if (m_curr_error > 3.14) {
            m_curr_error = -3.14;
        }
        else if (m_curr_error < -3.14) {
            m_curr_error = 3.14;
        }
        m_sum_error += m_curr_error;
        m_diff_error = m_curr_error - m_prev_error;

        ROS_INFO("Current error: %f", m_curr_error);
        
        m_ang_vel = m_kp*m_curr_error + m_ki*m_sum_error + m_kd*m_diff_error;

        // Bound m_ang_vel to +-m_elev_vel
        if (m_ang_vel > m_elev_vel) {
            m_ang_vel = m_elev_vel;
        }
        else if (m_ang_vel < m_elev_vel*-1) {
            m_ang_vel = m_elev_vel*-1;
        }

        // Publish the controlled velocity
        m_cmd_vel.angular.z = m_ang_vel;
        cmd_vel_pub.publish(m_cmd_vel);

        m_prev_error = m_curr_error;
    }

    // Close enough to turn goal. Stop robot.
    else {

        ROS_INFO("Reached goal angle.");

        // Stop
        m_cmd_vel.linear.x = 0.0;
        m_cmd_vel.angular.z = 0.0;
        cmd_vel_pub.publish(m_cmd_vel);

        // Reset PID control
        m_curr_error = 0.0;
        m_prev_error = 0.0;
        m_sum_error = 0.0;
        m_diff_error = 0.0;
        m_ang_vel = 0.0;

        // Increment motion control and got back to prev state
        m_motion_status++;
        m_state = m_prev_state;
    }
}


 /*
 * Provide HRI around exiting elevator.
 *
 * return: the list of possible floor IDs that the robot could be at on 
 * elevator exit
 */
FloorSet FSM::exit_elevator() {

    // Publish m_state to allow Sound Arduino to do it's thang
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = 3;
    audio_cmd_pub.publish(tmp);

    // Exit elevator by moving forward slowly
    // TODO: confirm this velocity
    m_cmd_vel.linear.x = m_elev_vel;
    cmd_vel_pub.publish(m_cmd_vel);

    // Once bot has exited elevator, stop
    float y_m_dist_traveled = abs(m_curr_pos.y - m_pos_old.y);
    if (y_m_dist_traveled > 2) {
        m_cmd_vel.linear.x = 0;
        cmd_vel_pub.publish(m_cmd_vel);
    }

    // TODO: Map matching behavior goes here.

    // TODO: write trigger for next m_state
    // if (scan_changes == 0) {
    //     m_state = ride_elevator();
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


void FSM::scan_callback(const sensor_msgs::LaserScan msg) {

    m_scan = msg.ranges;
}


void FSM::odom_callback(const nav_msgs::Odometry odom) {

    // Define current position and orientation
    m_curr_pos = odom.pose.pose.position;
    m_curr_quat = odom.pose.pose.orientation;

    // Get yaw angle
    m_curr_yaw = (float)tf::getYaw(m_curr_quat);
    
    //ROS_INFO("Yaw: %f\n", m_curr_yaw);
    //float prev_yaw = (float)tf::getYaw(m_quat_old);

    //// calculate distance traveled
    //float x_diff = FSM::m_curr_pos.x - FSM::m_pos_old.x;
    //float y_diff = FSM::m_curr_pos.y - FSM::m_pos_old.y;
    //m_dist_traveled = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    //// force yaws to always be within range [-pi, pi]
    //// see wrapPi(): 
    ////    https://github.com/ManickYoj/warmup_project_2017/blob/master/
    ////                  scripts/utils.py
    //float pi_yn = atan2(sin(m_curr_yaw), cos(m_curr_yaw));
    //float pi_yo = atan2(sin(prev_yaw), cos(prev_yaw));

    //// calculate angle rotated
    //// see diffAngle():
    ////    https://github.com/ManickYoj/warmup_project_2017/blob/master/
    ////                  scripts/utils.py
    //float d1 = pi_yn - pi_yo;
    //float d2 = 2*M_PI - abs(d1);
    //if (d1 > 0) d2 = d2 * -1.0;
    //if (abs(d1) < abs(d2)) m_ang_traveled = d1;
    //else m_ang_traveled = d2;

    //// set pose_new to old attrs
    //m_pos_old = m_curr_pos;
    //m_quat_old = m_curr_quat;
}


int main(int argc, char **argv) {

    ROS_INFO("Running main()");

    // Initialize ROS node
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle n;
    ros::Rate r(10); // rate to loop for (Hertz)

    // Init mission controller to first m_state on ground floor
    FSM mission_controller(n);   // init FSM
    mission_controller.m_state = booting; // set initial state
    mission_controller.m_floor.data = 1;

    // ROS Publishers
    audio_cmd_pub = n.advertise<std_msgs::Int8>("/audio_cmd", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // ROS Subscribers
    ros::Subscriber scan_sub = n.subscribe("/scan", 1000, &FSM::scan_callback,
            &mission_controller);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1000, &FSM::odom_callback,
            &mission_controller);

    while (ros::ok()) {

        // FSM decision tree
        switch(mission_controller.m_state) {

            case booting:
                mission_controller.boot();
                break;

            case exploring:
                mission_controller.explore_floor();
                break;

            case calling_elevator:
                mission_controller.call_elevator();
                break;

            case entering_elevator:
                mission_controller.enter_elevator();
                break;

            case riding_elevator:
                mission_controller.ride_elevator();
                break;

            case exiting_elevator:
                mission_controller.exit_elevator();
                break;

            case matching_map:
                mission_controller.match_map();
                break;

            case stopping:
                mission_controller.stop();
                break;

            case turning:
                mission_controller.turn_pid();
                break;

            default:
                break;
        }

        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Exiting mission_controller");
}
