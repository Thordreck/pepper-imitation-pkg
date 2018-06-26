#!/bin/bash
cd "$(dirname "$0")"

cd ../../..
source /opt/ros/kinetic/setup.bash
catkin_make
