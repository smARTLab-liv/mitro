#!/bin/bash
source /opt/ros/hydro/setup.bash
source /home/swarmlab/ros_catkin_ws/devel/setup.bash

DONE=0
while [ $DONE -lt 1 ] ; do
	if rosnode list
	then
		sleep 1
	        roslaunch --pid=/home/swarmlab/.ros/mitro_bringup_base.pid mitro_bringup mitro.launch
    		#roslaunch mitro_bringup mitro.launch
            DONE=1
	else
	        sleep 1
		echo "no roscore started... waiting"
	fi
done
