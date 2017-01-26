# RoboSys2017
Code base for the Mystery Machine autonomous race vehicle

---

## Project Description
The mystery machine autonomous race vehicle was created to autonomously navigate around the Olin Oval while avoiding static and dynamic obstacles.

Hardware Structure:

The mystery machine runs using an Odroid connected to an Arduino Mega to control motors, lights, and small emergency stop sensors and a powered usb hub connecting a suite of sensors including a LIDAR, camera, gps, and IMU.

Software Structure:

The software structure consists of three parts.  The hindbrain runs on the Arduino and controls immediate reaction to fast inputs such as perimeter sonar to allow emergency stopping before running into an unexpected obstacle and motor control.  Communicating with the Arduino using rosserial, the Odroid runs the ROS midbrain and forebrain.  The midbrain controls forward and backward motion, while the forebrain controls turning.  These values are sent over command velocity topics to a python arbiter script that chooses the most highly rated output command from the topics.  This is sent to the Arduino as the final velocity output.

---

## Running the Robot
There is a scripts folder in the mystery_machine package that contains scripts to run the appropriate launch files and commands for different functionalities include teleop mode, sensor data logging, navigation, etc. 

Current Functional Scripts:

* ports.sh - List all of the sensor port names to edit the launch files
* sensor_data_log.sh - Create a bag file with all topics
* telop_mode.sh - Run the robot with the following key layout:

```
	I
J	K 	L
	,

```
* race1-3.sh - Run the sensor and control nodes to autonomously race around the O

The following sections also detail running individual packages or sensor feeds from terminal.

---

## Individual Node and Sensor Documentation

NOTE: All of the ports in these commands follow the reassigned names set up on the ODROID.  To set up the port assignments on another machine run `./port_setup.sh` in the mystery_machine/scripts folder. Or to find the port names for terminal commands on another machine run `./ports.sh` in the Utils folder.

### Updating submodules

The funrobo_robot and phiget_drivers directories are independent git repositories. In order to update them, run:

`git submodule init && git submodule update`

### SSH Into the ODROID

`ssh odroid@192.168.16.68`

### Teleop Mode

``rosrun rosserial_python serial_node.py /dev/ttyUSB0`
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

### Sensors

**LIDAR**

To initialize and visualize the LIDAR scans on /scan run:

`sudo chmod a+rw /dev/ttyACM1`

`rosrun hokuyo_node hokuyo_node /dev/ttyACM1`

`rosrun rviz rviz`

Add a LaserScan and set the topic to /scan.  Change fixed frame from map to laser.

**GPS**

To start the GPS feed on /fix run:

`sudo chmod a+rw /dev/ttyACM0`

`rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyACM0 _baud:=115200`

**IMU**

To start the imu feed run:

`rosrun phidgets_imu phidgets_imu_node`

**Camera**

To start the camera feed run:

rosrun uvc_camera uvc_camera_node _device:="/dev/video0"

**Rosserial**

If the Arduino is running, it will listen for /cmd_vel.

`rosrun rosserial_python serial_node.py /dev/ttyUSB0`

### Forebrain Controller
rosrun myster_machine forebrain

### Midbrain Controller
rosrun mystery_machine midbrain

### Hindbrain Controller
rosrun mystery_machine arbiter.py

### HW 3 Demo

The homework 3 demo can:

* Use the LIDAR to detect obstacles in front of the robot and slow or stop in response
* Use the GPS to find the robot's location in reference to a waypoint and command the robot to turn toward the waypoint
* Use rosserial to pass messages on the /cmd_vel topic from the Midbrain Arbiter (compiles the command velocities from the previous two functions) to the Arduino to move the motors

**To launch this demo run:**

`roslaunch mystery_machine hw3demo.launch`