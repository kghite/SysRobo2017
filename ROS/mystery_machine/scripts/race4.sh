#!/bin/bash

echo "Beginning Race 4 Mode"

sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM1
sudo chmod a+rw /dev/video0
sudo chmod a+rw /dev/ttyACM2

roslaunch mystery_machine race4.launch