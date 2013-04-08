#!/bin/bash
source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=/home/swarmlab/ros:$ROS_PACKAGE_PATH

DONE=0
while [ $DONE -lt 1 ] ; do
	if rosnode list
	then
		sleep 1
	        roslaunch --pid=/home/swarmlab/mitro_bringup_base.pid mitro_bringup base.launch
    		DONE=1
	else
	        sleep 1
		echo "no roscore started... waiting"
	fi
done
