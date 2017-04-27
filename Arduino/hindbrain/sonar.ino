
// Variables for panning sonar servo and sonar sensor
const uint8_t SONAR_PIN = A7;
const uint8_t SONAR_PAN_PIN = 45;
Adafruit_TiCoServo sonar_pan_servo;
uint8_t sonar_pan_angle = 90;
unsigned long time_of_last_sonar_pan = millis();
uint8_t sonar_pan_time_interval = 15;
uint8_t sonar_pan_angle_increment = 1;
float sonar_reading;
unsigned int sonar_point_id = 0;


// Setup process for sonar
void setup_sonar() {
  
  sonar_pan_servo.attach(SONAR_PAN_PIN);
}


// Move the sonar servo in timed increments and publish a new reading from the sonar
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
    
    // Denote that the servo was last moved at the current time
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
