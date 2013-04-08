#!/bin/bash
source /opt/ros/fuerte/setup.bash
export ROS_MASTER_URI=http://bob:11311
export ROS_PACKAGE_PATH=/home/swarmlab/ros:$ROS_PACKAGE_PATH

DONE=0
while [ $DONE -lt 1 ] ; do
	echo waiting for network connection...
	ping -c1 10.10.14.1 >/dev/null 2>&1
	if [ $? -eq 0 ]
	then
		roslaunch --pid=/home/swarmlab/mitro_bringup_laptop.pid mitro_bringup laptop.launch
		DONE=1
	fi
done
