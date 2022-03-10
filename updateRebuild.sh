#!/bin/bash
sudo apt update
sudo apt upgrade -y
wstool update -t src
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release 
