#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <Adafruit_TiCoServo.h>
#include <mystery_machine/SonarScan.h>


#define USB_CON
#define PI 3.14159265358979323846


// Timing variable
unsigned long current_time;

// Pins and servo objects for Roboclaw control
const uint8_t FORWARD_PIN = 6;
const uint8_t TURN_PIN = 7;
Adafruit_TiCoServo forward_channel;
Adafruit_TiCoServo turn_channel;
int linear_vel =  0;
int angular_vel = 0;
int current_linear_vel = 0;
int current_angular_vel = 0;

// Define estop pin
const uint8_t ESTOP_PIN = 42;

// Define IR sensor variables
const uint8_t IR_PIN_1 = A0;
const uint8_t IR_PIN_2 = A1;
const int ir_low_threshold  = 300;
uint8_t ir_estop = 0;

// Define LIDAR tilt servo
const uint8_t LIDAR_TILT_PIN = 8;
Adafruit_TiCoServo lidar_tilt_servo;
const uint8_t middle_tilt_position = 110;
int current_tilt_position = middle_tilt_position;


// ROS node handle
ros::NodeHandle nh;


// ROS publishers
std_msgs::String debug_msg;
ros::Publisher debug_pub("debug", &debug_msg);

std_msgs::Int16 ir_estop_msg;
ros::Publisher ir_estop_pub("ir_estop", &ir_estop_msg);


// Callback function for a Twist message
void cmd_vel_callback( const geometry_msgs::Twist& cmd_vel ) {
  
  // Extract velocity data
  // Multiply by 90 to maintain resolution
  // Ends up giving a signal for write between 0-180
  // Make sure cmd_vel has values that range -1 to 1
  linear_vel  = int(90 * cmd_vel.linear.x);
  angular_vel = int(90 * cmd_vel.angular.z);
  
  // Print the received Twist message
  String debug_info = "Received Twist message!\n";
  debug_info += "Linear vel: " + String(linear_vel) + "\n";
  debug_info += "Angular vel: " + String(angular_vel);
  debug_print(debug_info);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback );


// Callback function for an IR Estop message
void ir_estop_callback(const std_msgs::Int16& msg) {
  
  // Check and see if the midbrain says it's okay to move
  int estop_message = msg.data;
  
  if (estop_message == 2){
    ir_estop = 2;
  }
  
}
ros::Subscriber<std_msgs::Int16> ir_estop_sub("ir_estop",&ir_estop_callback);


// Callback function for an Odroid Estop message
int odroid_estop = 0;
void odroid_estop_callback(const std_msgs::Int16& msg) {
  
  // Just update this estop.. Pretty easy
  int estop_message = msg.data;
  
  odroid_estop = estop_message;
}
ros::Subscriber<std_msgs::Int16> odroid_estop_sub("odroid_estop",&odroid_estop_callback);


// SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  
  // ROS Node
  nh.initNode();
  
  // ROS publishers
  nh.advertise(debug_pub);
  nh.advertise(ir_estop_pub);
  
  // ROS subscribers
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(ir_estop_sub);
  nh.subscribe(odroid_estop_sub);
  
  // Initialize timing
  current_time = millis();

  // Attach Servo objects to correct pins
  forward_channel.attach(FORWARD_PIN);
  turn_channel.attach(TURN_PIN);
  lidar_tilt_servo.attach(LIDAR_TILT_PIN);
  
  // System setup
  setup_encoders();
  setup_sonar();
  setup_lights();
  setup_audio();
}


// Run hindbrain loop until commanded to stop LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  
  // Run loop functions
  check_ir_sensors();
  update_drive_motors();
  update_pan_and_read_sonar();
  check_encoders();
  blink_lights();
  
  // Run ros callbacks
  nh.spinOnce();
  
  // Sleep briefly to not rail processing
  delay(1);
}


// Hindbrain Helper Functions******************************************************************************

// Writes a String message to the /debug topic
void debug_print(String message) {
  
  char charBuf[100];
  message.toCharArray(charBuf,100);    
  debug_msg.data = charBuf;
  debug_pub.publish(&debug_msg);
}


// Send current linear and angular velocity commands to the drive motors
void update_drive_motors() {
  
  // The ir_ is the only estop the Arduino has to tell itself to stop via software
  if (ir_estop == 1 || odroid_estop == 1) {
   forward_channel.write(90);
    turn_channel.write(90);
    current_linear_vel = 0;
    current_angular_vel = 0;
  }
  else {
    // Write velocities to drive motors
    forward_channel.write(90 + current_linear_vel);
    turn_channel.write(90 - current_angular_vel);
    
    // Slowly ramp current velocities to a setpoint value
    if (linear_vel != current_linear_vel) {
      current_linear_vel += (linear_vel > current_linear_vel) ? 1 : -1;
    }
    if (angular_vel != current_angular_vel) {
      current_angular_vel += (angular_vel > current_angular_vel) ? 1 : -1;
    }
  }
  
  // Print the state
//  debug_info = "Updating drive motors!";
//  debug_info += "Linear vel goal: " + String(linear_vel) + "\n";
//  debug_info += "Linear vel: " + String(current_linear_vel) + "\n";
//  debug_info += "Angular vel goal: " + String(angular_vel) + "\n";
//  debug_info += "Angular vel: " + String(current_angular_vel);
//  debug_print(debug_info);
}



// See if we need to estop based on IR input
void check_ir_sensors() {
  
  int ir_reading_1;
  int ir_reading_2;
  ir_reading_1 = analogRead(IR_PIN_1);
  ir_reading_2 = analogRead(IR_PIN_2);
  
  if (ir_reading_1 < ir_low_threshold || ir_reading_2 < ir_low_threshold) {
    if (ir_estop == 0) {
      ir_estop = 1;
      ir_estop_msg.data = ir_estop;
      ir_estop_pub.publish(&ir_estop_msg);
    }
    else if (ir_estop == 2 && linear_vel > 0) {
      ir_estop = 1;
      ir_estop_msg.data = ir_estop;
      ir_estop_pub.publish(&ir_estop_msg);
    }
  }
  else if (ir_estop != 0) {
    ir_estop = 0;
    ir_estop_msg.data = ir_estop;
    ir_estop_pub.publish(&ir_estop_msg);
  }
}


// Helper function for attaching an interrupt to a pin
int digital_pin_to_interrupt(int pin) {
  
  switch(pin) {
    case 2:
      return 0;
    case 3:
      return 1;
    case 18:
      return 5;
    case 19:
      return 4;
    case 20:
      return 3;
    case 21:
      return 2;
  }
}
