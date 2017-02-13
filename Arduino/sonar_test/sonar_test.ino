/*
Drives Servo and thing and publishes things
*/

#include <Servo.h>

Servo sonarservo;  // create servo object to control a servo

int ultrasonicpin = 0;  // analog pin used to connect the potentiometer
int pos = 0; //position of servo in degrees
int rawultrasound;    // variable to read the value from the analog pin
float distance; //variable to store computed distance



void setup() {
  sonarservo.attach(36);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); //start serial communication
}

void loop() {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    sonarservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    rawultrasound = analogRead(ultrasonicpin); // reads the value of the potentiometer (value between 0 and 1023)
    rawultrasound = rawultrasound / 100.0;
    Serial.println(pos,rawultrasound);
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    sonarservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position 
    rawultrasound = analogRead(ultrasonicpin);            // reads the value of the potentiometer (value between 0 and 1023)
  }
}

