#!/bin/bash

echo "Beginning LIDAR Following Mode"

sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM1

roslaunch mystery_machine lidar_following.launch
