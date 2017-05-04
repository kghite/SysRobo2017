# RoboSys2017
Code base for the Mystery Machine autonomous exploration vehicle


## Project Description

The Mystery Machine exploration vehicle was created to autonomously exlpore an unknown building by assembling maps of every floor in the building, and navigating through elevators to different floors. This was for the Integrated Robotic Systems class at Olin College of Engineering run by Drew Bennett and Dave Barrett in the spring of 2017. For this project, we built off of a previous robot base, which you can view the original code for [here](https://github.com/kghite/FunRobo2016).

## Architecture

#### Hardware Structure:

The mystery machine runs using an Odroid connected to an Arduino Mega to control motors, lights, and small emergency stop sensors and a powered usb hub connecting a suite of sensors including a LIDAR, sonar,camera, gps, and IMU.

For a more complete structuring of our electrical system, see the diagram below:
![Electrical Diagram](https://github.com/kghite/SysRobo2017/blob/master/wiring_diagram.png)

#### Software Structure:

The software structure consists of two parts.  The hindbrain runs on the Arduino and controls immediate reaction to fast inputs, such as perimeter sonar sensors (to allow emergency stopping before running into an unexpected obstacle or down stairs) and motor control. Communicating with the Arduino using rosserial, the Odroid runs the ROS Finite State Machine. The FSM, which you can see laid out in the diagram below, determines which state the robot is in, manages the transitions between states, and calls whichever scripts should be running to execute that state's mission. Any movement of the robot is piloted by whichever state it is in, and is sent over command velocity topics to the Arduino as the velocity output.

![Finite State Diagram](https://github.com/kghite/SysRobo2017/blob/master/finite_state_diagram_final.png)

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

#### Teleop Mode

`roslaunch mystery_machine teleop.launch arduino_port:=<ARDUINO_PORT>`

#### Data-Logging Mode

`roslaunch mystery_machine data_log.launch arduino_port:=<ARDUINO_PORT> lidar_port:=<LIDAR_PORT> camera_port:=<CAMERA_PORT>`

#### Mapping Mode

`roslaunch mystery_machine mapping.launch arduino_port:=<ARDUINO_PORT> lidar_port:=<LIDAR_PORT>`


## Individual Node and Sensor Documentation

NOTE: To find the port names for terminal commands on another machine run `./ports.sh` in the `/scripts` folder.

#### Rosserial

If the Arduino is running, it will listen for `/cmd_vel`.

`rosrun rosserial_python serial_node.py <ARDUINO_PORT>`

#### LIDAR

To initialize and visualize the LIDAR scans on `/scan` run:

```
sudo chmod a+rw <LIDAR_PORT>
rosrun hokuyo_node hokuyo_node <LIDAR_PORT>
rosrun rviz rviz
```

Add a LaserScan and set the topic to /scan.  Change fixed frame from map to laser.

#### GPS

To start the GPS feed on `/fix` run:

```
sudo chmod a+rw <GPS_PORT>
rosrun nmea_navsat_driver nmea_serial_driver _port:=<GPS_PORT> _baud:=115200
```

#### IMU

To start the imu feed run:

`rosrun phidgets_imu phidgets_imu_node`

#### Camera

To start the camera feed run:

`rosrun uvc_camera uvc_camera_node _device:=<CAMERA_PORT`
