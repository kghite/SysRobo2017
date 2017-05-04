# RoboSys2017
Code base for the Mystery Machine autonomous race vehicle


## Project Description

The mystery machine autonomous race vehicle was created to autonomously navigate around the Olin Oval while avoiding static and dynamic obstacles.

#### Hardware Structure:

The mystery machine runs using an Odroid connected to an Arduino Mega to control motors, lights, and small emergency stop sensors and a powered usb hub connecting a suite of sensors including a LIDAR, camera, gps, and IMU.

#### Software Structure:

The software structure consists of three parts.  The hindbrain runs on the Arduino and controls immediate reaction to fast inputs such as perimeter sonar to allow emergency stopping before running into an unexpected obstacle and motor control.  Communicating with the Arduino using rosserial, the Odroid runs the ROS midbrain and forebrain.  The midbrain controls forward and backward motion, while the forebrain controls turning.  These values are sent over command velocity topics to a python arbiter script that chooses the most highly rated output command from the topics.  This is sent to the Arduino as the final velocity output.


## Our Scripts

There is a scripts folder in the mystery_machine package that contains scripts to run the appropriate launch files and commands for different functionalities include teleop mode, sensor data logging, navigation, etc. 

Current Functional Scripts:

- ports.sh - List all of the sensor port names to edit the launch files
- telop_mode.sh - Run the robot with the following key layout:

```
	I
J	K 	L
	,
```

The following sections also detail running individual packages or sensor feeds from terminal.


## Setup

#### Updating submodules

Our repository contains other individual git repositories. To pull these repositories, run the following:

`git submodule init && git submodule update`

#### Installing ROS Dependencies

Install all necessary rospackages and other dependencies for the `mystery_machine` package.

` rosdep install -r --from-paths .`

#### Building rosserial_arduino libraries

Remove any pre-existing `ros_lib` directory.

`rm -r <PATH_TO_ros_lib>`

Typically, you can find `ros_lib` in `~/sketchbook/libraries`, so this command would be:

`rm -r ~/sketchbook/libraries/ros_lib`

Now, create the new `ros_lib` directory.

`rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries`


## Running the Robot

#### SSH Into the Odroid

**WiFi**

Make sure you are connected to the OLIN-ROBOTICS WiFi network.

`ssh odroid@192.168.16.83`

**Ethernet**

Scan for the correct IP address using the eth0 IP address from `ifconfig`.

`nmap ##.##.##.0/24`

SSH into the IP address with port 22 open. It may take a while to see the open SSH port.

`ssh odroid@<IP_ADDRESS_FOUND_WITH_NMAP>`

#### Startup the robot

`roslaunch mystery_machine bringup.launch`

This command will start:

- Rosserial communication with the Arduino Mega
- Lidar Hokuyo node
- Transform for the lidar sensor
- Transform for the sonar sensor
- SonarTransform node that merges sonar and lidar scans

#### Teleoperation

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

This will allow the user to control the robot's movement with keyboard input.

#### Mapping

First run `bringup.launch` from earlier.

`roslaunch mystery_machine mapping.launch`

Maps, with glass-detection included, will be automatically generated from the merged lidar and sonar scan.

#### Visualization

Open rviz with any of our custom configuration files to view results of the above software.

`rviz -d rviz/<CONFIG_FILE_NAME>`

for example

`rviz -d rviz/mapping.rviz`
