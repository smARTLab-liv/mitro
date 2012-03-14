#!/bin/bash
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=/home/swarmlab/ros:$ROS_PACKAGE_PATH

DONE=0
while [ $DONE -lt 1 ] ; do
	if ps ax | grep -v grep | grep roscore > /dev/null
	then
		roslaunch --pid=/home/swarmlab/mitro_bringup_base.pid mitro_bringup base.launch
    		DONE=1
	else
		echo "no roscore started... waiting"
	fi
done
