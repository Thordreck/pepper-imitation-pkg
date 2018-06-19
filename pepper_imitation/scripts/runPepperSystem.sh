#!/bin/bash
cd "$(dirname "$0")"

source /opt/ros/kinetic/setup.bash

find ../../.. -type d -name "devel" 2> /dev/null | while read line; do
	if source $line/setup.bash 2> /dev/null; then
		roslaunch pepper_imitation pepper_imitation.launch
		exit 0
	fi
done  

echo "Could not find build folder"
exit 1
