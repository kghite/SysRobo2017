/*
 * File: mission_control.cpp
 * 
 * Updated: 3/29/17
 * 
 * Description: ROS integrated FSM for building mapping and elevator
 * exploration with the Mystery Machine  
*/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>

#include "utils.h"

// declaring publishers
ros::Publisher audio_cmd_pub;
ros::Publisher cmd_vel_pub;

// Set up m_state values
enum State {
    booting,
    calling_elevator,
    entering_elevator,
    riding_elevator,
    exiting_elevator,
    stopping,
    turning,
    testing
};


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

        // Declaring constructor for FSM, which takes arg 'ROS node handler'
        FSM(ros::NodeHandle n);

        // Declaring all msg types
        geometry_msgs::PoseStamped m_current_pose;
        geometry_msgs::Pose m_goal_pose;
        geometry_msgs::Twist m_cmd_vel;
        //nav_msgs::Odometry m_odom;
        std_msgs::Int8 m_floor;

        // Declare methods
        void boot();
        void call_elevator();
        void enter_elevator();
        void ride_elevator();
        void stop();
        void turn_pid();
        void test();
        void exit_elevator();

        // Declare callback methods
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

        // PID
        float m_curr_error = 0.0;
        float m_prev_error = 0.0;
        float m_sum_error = 0.0;
        float m_diff_error = 0.0;
        float m_ang_vel = 0.0;
        float m_elevator_yaw = 0.78;

        // FSM::ride_elevator
        float m_closed_mid_elev_scan_avg = 0.0;
        uint8_t m_mid_elev_open = 0;
};


/*
* This is equivalnt to the init() method in python.
*/
FSM::FSM(ros::NodeHandle n) {
    m_elev_vel = 0.1; // speed for moving around the elevator
    m_scan = std::vector <float> (512, 0.0); // fill scan with zeros
    m_motion_status = 0; // Init motion sequence counter
}


void FSM::boot() {

    ROS_INFO("Booting");

    // Make sure we don't play sound from a previous session
    std_msgs::Int8 tmp = std_msgs::Int8();
    tmp.data = 4;
    audio_cmd_pub.publish(tmp);

    if (m_loops_to_spend_in_boot - m_loops_spent_in_boot <= 0) {

        // Test PID control
        m_goal_yaw = m_curr_yaw - 1.57;
        m_prev_state = stopping;
        m_state = turning;

        // Start at elevator interaction
        //m_state = calling_elevator;

        //m_state = testing;
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
            m_cmd_vel.linear.x = m_elev_vel;
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
            m_cmd_vel.linear.x = m_elev_vel;
            cmd_vel_pub.publish(m_cmd_vel);
            ROS_INFO("Forward scan: %f", m_scan.at(256));
            if (m_scan.at(256) < 0.5) {
                m_cmd_vel.linear.x = 0.0;
                cmd_vel_pub.publish(m_cmd_vel);
                m_motion_status++;
            }
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

    // Constrain lidar scan to front
    std::vector<float> mid_elev_scan(m_scan.begin()+170,
            m_scan.begin()+341);

    // Find average distance on left and right sides of LaserScan
    float avg_mid_elev_scan = avg(mid_elev_scan);

    // Debug print statements
    ROS_INFO("Closed elev scan avg: %f", m_closed_mid_elev_scan_avg);
    ROS_INFO("Avg right elevator scan: %f", avg_mid_elev_scan);

    // If a baseline for what the LaserScan looks like when the elevators are
    // closed has not been defined, then define it
    if (m_closed_mid_elev_scan_avg == 0.0) {

        m_closed_mid_elev_scan_avg = avg_mid_elev_scan;
    }

    // If the elevator door opened
    if (avg_mid_elev_scan - m_closed_mid_elev_scan_avg >=
            m_elev_open_thresh) {

        ROS_INFO("Door opened");
        m_mid_elev_open = 1;
        m_state = exiting_elevator;
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
    m_cmd_vel.linear.x = m_elev_vel;
    cmd_vel_pub.publish(m_cmd_vel);

    ros::Duration(1.0).sleep();

    m_cmd_vel.linear.x = 0;
    cmd_vel_pub.publish(m_cmd_vel);
    m_state = stopping;

    // Once bot has exited elevator, stop
    // float x_m_dist_traveled = abs(m_curr_pos.x - m_pos_old.x);

    // if (x_m_dist_traveled > 1.5) {
    //     m_cmd_vel.linear.x = 0;
    //     cmd_vel_pub.publish(m_cmd_vel);
    //     m_state = stopping;
    // }
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

    float kp = 0.09;
    float ki = 0.004;
    float kd = 0.0;
    float p_term;
    float i_term;
    float d_term;
    float sum_error_max = .06;
    float yaw_thresh = 0.05; // in radians

    // Use PID control to turn until close enough to the turn goal
    if (abs(m_goal_yaw - m_curr_yaw) > yaw_thresh) {

        m_curr_error = get_relative_diff_radians(m_curr_yaw, m_goal_yaw);
        m_sum_error += m_curr_error;
        m_diff_error = m_curr_error - m_prev_error;

        p_term = kp*m_curr_error;
        i_term = bound_float(ki*m_sum_error, -sum_error_max, sum_error_max);
        d_term = kd*m_diff_error;

        m_ang_vel = p_term + i_term + d_term;

        // Bound m_ang_vel to +-m_elev_vel
        if (m_ang_vel > m_elev_vel) {
            m_ang_vel = m_elev_vel;
        }
        else if (m_ang_vel < m_elev_vel*-1) {
            m_ang_vel = m_elev_vel*-1;
        }

        ROS_INFO("Current error: %f", m_curr_error);
        ROS_INFO("P term: %f", p_term);
        ROS_INFO("I term: %f", i_term);
        ROS_INFO("D term: %f", d_term);
        ROS_INFO("Ang vel: %f", m_ang_vel);

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
 * Testing state for development.
 */
void FSM::test() {

    ROS_INFO("Testing.");

    float angle = 3*M_PI;
    ROS_INFO("Angle before wrapping: %f", angle);
    ROS_INFO("Angle after wrapping: %f", wrap_radians(angle));

    angle = -4*M_PI;
    ROS_INFO("Angle before wrapping: %f", angle);
    ROS_INFO("Angle after wrapping: %f", wrap_radians(angle));

    float ang_1 = M_PI/4;
    float ang_2 = 7*M_PI/4;
    ROS_INFO("Diff between %f and %f: %f", ang_1, ang_2,
            get_relative_diff_radians(ang_1, ang_2));

    m_state = stopping;
}


void FSM::scan_callback(const sensor_msgs::LaserScan msg) {

    m_scan = msg.ranges;
}


void FSM::odom_callback(const nav_msgs::Odometry odom) {

    // Set past postition
    m_pos_old = m_curr_pos;
    m_quat_old = m_curr_quat;

    // Define current position and orientation
    m_curr_pos = odom.pose.pose.position;
    m_curr_quat = odom.pose.pose.orientation;

    // Get yaw angle
    m_curr_yaw = wrap_radians((float)tf::getYaw(m_curr_quat));
    // Change m_curr_yaw from (-pi to pi) to (0 to 2pi)
    if (m_curr_yaw < 0.0) {
        m_curr_yaw += 2 * M_PI;
    }
}


int main(int argc, char **argv) {

    ROS_INFO("Running main");

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

            case stopping:
                mission_controller.stop();
                break;

            case turning:
                mission_controller.turn_pid();
                break;

            case testing:
                mission_controller.test();
                break;

            default:
                break;
        }

        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO("Exiting mission_controller");
}
