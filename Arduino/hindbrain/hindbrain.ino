#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
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
const byte FORWARD_PIN = 6;
const byte TURN_PIN = 7;
Adafruit_TiCoServo forward_channel;
Adafruit_TiCoServo turn_channel;
int linear_vel =  0;
int angular_vel = 0;
int current_linear_vel = 0;
int current_angular_vel = 0;

// Variables for panning sonar servo and sonar sensor
const byte SONAR_PIN = A7;
const byte SONAR_PAN_PIN = 45;
Adafruit_TiCoServo sonar_pan_servo;
int sonar_pan_angle = 90;
unsigned long time_of_last_sonar_pan = millis();
int sonar_pan_time_interval = 15;
int sonar_pan_angle_increment = 1;
float sonar_reading;
int sonar_point_id = 0;

// Define estop pin
const byte ESTOP_PIN = 42;

// Define IR sensor variables
const byte IR_PIN_1 = A0;
const byte IR_PIN_2 = A1;
const int ir_low_threshold  = 300;
int ir_estop = 0;

// Motor encoder globals
int left_encoder_pin_A = 18;
int left_encoder_pin_B = 19;
long left_encoder_pos = 0;
boolean left_encoder_updated = false;
int right_encoder_pin_A = 20;
int right_encoder_pin_B = 21;
long right_encoder_pos = 0;
boolean right_encoder_updated = false;
const int MAX_ENCODER_VAL = 32767;
const int MIN_ENCODER_VAL = -32768;

// Define LIDAR tilt servo
const byte LIDAR_TILT_PIN = 8;
Adafruit_TiCoServo lidar_tilt_servo;
const int middle_tilt_position = 110;
int current_tilt_position = middle_tilt_position;


// Set up ROS node handling and feedback channel
ros::NodeHandle nh;


// Set up ROS publishers
std_msgs::String debug_msg;
ros::Publisher debug_publisher("debug", &debug_msg);

std_msgs::Int16 ir_estop_msg;
ros::Publisher ir_estop_publisher("ir_estop", &ir_estop_msg);

mystery_machine::SonarScan sonar_data_msg;
ros::Publisher sonar_data_publisher("sonar_data", &sonar_data_msg);

std_msgs::Int16 left_encoder_msg;
ros::Publisher left_encoder_publisher("lwheel", &left_encoder_msg);

std_msgs::Int16 right_encoder_msg;
ros::Publisher right_encoder_publisher("rwheel", &right_encoder_msg);


// Callback function for a Twist message
void cmd_vel_callback( const geometry_msgs::Twist& cmd_vel ) {
  
  // Extract velocity data
  // Multiply by 90 to maintain resolution
  // Ends up giving a signal for write between 0-180
  // Make sure cmd_vel has values that range -1 to 1
  linear_vel  = int(90 * cmd_vel.linear.x);
  angular_vel = int(90 * cmd_vel.angular.z);
  
  // Print the received Twist message
//  debug_info = "Received Twist message!\n";
//  debug_info += "Linear vel: " + String(linear_vel) + "\n";
//  debug_info += "Angular vel: " + String(angular_vel);
//  debug_print(debug_info);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback );


// Callback function for an IR Estop message
void ir_estop_callback( const std_msgs::Int16& msg ) {
  
  // Check and see if the midbrain says it's okay to move
  int estop_message = msg.data;
  
  if (estop_message == 2){
    ir_estop = 2;
  }
  
}
ros::Subscriber<std_msgs::Int16>ir_estop_sub("ir_estop",&ir_estop_callback);


// Callback function for an Odroid Estop message
int odroid_estop = 0;
void odroidEstopCallback( const std_msgs::Int16& msg ) {
  
  // Just update this estop.. Pretty easy
  int estop_message = msg.data;
  
  odroid_estop = estop_message;
}
ros::Subscriber<std_msgs::Int16>odroid_estop_sub("odroid_estop",&odroidEstopCallback);


// SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup() {
  
  // Initialize timing things
  current_time = millis();
  
  setup_lights();
  
  // Define pin modes
  pinMode(ESTOP_PIN, INPUT);
  pinMode(SONAR_PAN_PIN, INPUT);
  
  // Setup encoders and encoder interrupts
  pinMode(left_encoder_pin_A, INPUT);
  digitalWrite(left_encoder_pin_A, HIGH);
  pinMode(left_encoder_pin_B, INPUT);
  digitalWrite(left_encoder_pin_B, HIGH);
  attachInterrupt(digital_pin_to_interrupt(left_encoder_pin_A), update_left_encoder, CHANGE);
  pinMode(right_encoder_pin_A, INPUT);
  digitalWrite(right_encoder_pin_A, HIGH);
  pinMode(right_encoder_pin_B, INPUT);
  digitalWrite(right_encoder_pin_B, HIGH);
  attachInterrupt(digital_pin_to_interrupt(right_encoder_pin_A), update_right_encoder, CHANGE);
  
  // Attach Servo objects to correct pins
  forward_channel.attach(FORWARD_PIN);
  turn_channel.attach(TURN_PIN);
  lidar_tilt_servo.attach(LIDAR_TILT_PIN);
  sonar_pan_servo.attach(SONAR_PAN_PIN);
  
  // Initialize ROS topics
  nh.initNode();
  nh.advertise(debug_publisher);
  nh.advertise(ir_estop_publisher);
  nh.advertise(sonar_data_publisher);
  nh.advertise(left_encoder_publisher);
  nh.advertise(right_encoder_publisher);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(ir_estop_sub);
  nh.subscribe(odroid_estop_sub);
}


// Run hindbrain loop until commanded to stop LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop() {
  
  // Run loop functions
  check_ir_sensors();
  update_drive_motors();
  update_pan_and_read_sonar();
  blink_lights();
  check_encoders();
  
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
  debug_publisher.publish(&debug_msg);
}


// Check if new encoder values have been read - if so, publish them
void check_encoders() {
  
  check_left_encoder();
  check_right_encoder();
}


// Check if new left_encoder values have been read - if so, publish them
void check_left_encoder() {
  
  if (left_encoder_updated) {
    left_encoder_msg.data = left_encoder_pos;
    left_encoder_publisher.publish(&left_encoder_msg);
    left_encoder_updated = false;
  }
}


// Check if new right_encoder values have been read - if so, publish them
void check_right_encoder() {
  
  if (right_encoder_updated) {
    right_encoder_msg.data = right_encoder_pos;
    right_encoder_publisher.publish(&right_encoder_msg);
    right_encoder_updated = false;
  }
}


void update_left_encoder() {
  
  int channel_A = digitalRead(left_encoder_pin_A);
  int channel_B = digitalRead(left_encoder_pin_B);
  if (channel_A == channel_B) {
    // Decrement encoder value unless it is at min, in which case it should wrap
    if (left_encoder_pos <= MIN_ENCODER_VAL) {
      left_encoder_pos = MAX_ENCODER_VAL;
    }
    else {
      left_encoder_pos--;
    }
  }
  else {
    // Increment encoder value unless it is at max, in which case it should wrap
    if (left_encoder_pos >= MAX_ENCODER_VAL) {
      left_encoder_pos = MIN_ENCODER_VAL;
    }
    else {
      left_encoder_pos++;
    }
  }
  
  left_encoder_updated = true;
}


void update_right_encoder() {
  
  int channel_A = digitalRead(right_encoder_pin_A);
  int channel_B = digitalRead(right_encoder_pin_B);
  if (channel_A == channel_B) {
    // Increment encoder value unless it is at max, in which case it should wrap
    if (right_encoder_pos >= MAX_ENCODER_VAL) {
      right_encoder_pos = MIN_ENCODER_VAL;
    }
    else {
      right_encoder_pos++;
    }
  }
  else {
    // Decrement encoder value unless it is at min, in which case it should wrap
    if (right_encoder_pos <= MIN_ENCODER_VAL) {
      right_encoder_pos = MAX_ENCODER_VAL;
    }
    else {
      right_encoder_pos--;
    }
  }
  
  right_encoder_updated = true;
}


// Update motor speeds
void update_drive_motors() {
  
  // The ir_estop is the only estop the Arduino has to tell itself to stop via software
  if (ir_estop == 1 || odroid_estop == 1){
   forward_channel.write(90);
    turn_channel.write(90);
    current_linear_vel = 0;
    current_angular_vel = 0;
  }
  else{
    forward_channel.write(90 + current_linear_vel);
    turn_channel.write(90 - current_angular_vel);
    
    if (linear_vel > current_linear_vel) {
      current_linear_vel += 1;
    }
    else if (linear_vel < current_linear_vel) {
      current_linear_vel -= 1;
    }
      
    if (angular_vel > current_angular_vel) {
      current_angular_vel += 1;
    }
    else if (angular_vel < current_angular_vel) {
      current_angular_vel -= 1;
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


void update_pan_and_read_sonar() {

  // If enough time has passed, move the sonar pan servo
  current_time = millis();
  if (current_time - time_of_last_sonar_pan > sonar_pan_time_interval) {
    
    // Increment servo angle
    sonar_pan_angle += sonar_pan_angle_increment;
    
    // If the sonar pan servo has reached the end of a sweep, change direction
    if (sonar_pan_angle <= 0) {
      sonar_pan_angle_increment = 1;
      sonar_pan_angle = 0;
    }
    else if (sonar_pan_angle >= 180) {
      sonar_pan_angle_increment = -1;
      sonar_pan_angle = 180;
    }

    // Write the new angle to the servo
    sonar_pan_servo.write(sonar_pan_angle);
    time_of_last_sonar_pan = millis();
  }
  // Publish sonar data
  sonar_reading = analogRead(SONAR_PIN)/100.0;
  sonar_point_id ++;
  sonar_data_msg.header.seq = sonar_point_id;
  sonar_data_msg.header.stamp.sec = millis()/1000;
  sonar_data_msg.header.stamp.nsec = (millis() * 1000) % 1000000;
  sonar_data_msg.header.frame_id = "base_sonar";
  sonar_data_msg.range_min = 0.05;
  sonar_data_msg.range_max = 2.00;
  sonar_data_msg.range = sonar_reading;
  sonar_data_msg.angle = sonar_pan_angle; // angle in degrees where 0 is right and 180 is left
  sonar_data_publisher.publish(&sonar_data_msg);
}


// See if we need to estop based on IR input
void check_ir_sensors() {
  
  int ir_reading_1;
  int ir_reading_2;
  ir_reading_1 = analogRead(IR_PIN_1);
  ir_reading_2 = analogRead(IR_PIN_2);
  
  if (ir_reading_1 < ir_low_threshold || ir_reading_2 < ir_low_threshold){
    if (ir_estop == 0){
      ir_estop = 1;
      ir_estop_msg.data = ir_estop;
      ir_estop_publisher.publish(&ir_estop_msg);
    }
    
    else if (ir_estop == 2){
      
      if (linear_vel > 0){
        ir_estop = 1;
        ir_estop_msg.data = ir_estop;
        ir_estop_publisher.publish(&ir_estop_msg);
      }
    }
  }
  else{
    if (ir_estop != 0){
      ir_estop = 0;
      ir_estop_msg.data = ir_estop;
      ir_estop_publisher.publish(&ir_estop_msg);
    }
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
