#!/bin/bash
sudo apt update
sudo apt upgrade -y
wstool update -t src
# dirty-fix: nimbro_network cmake file formating
sed -i 's/cmake_minimum_required(VERSION 3.10)/cmake_minimum_required(VERSION 3.10.0)/g' \
    ./src/omniveyor/nimbro_network/nimbro_network/CMakeLists.txt
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release 
