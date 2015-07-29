#!/bin/bash
source /opt/ros/indigo/setup.bash
source /home/smartlab/ros_ws/devel/setup.bash

DONE=0
while [ $DONE -eq 0 ] ; do
    RUNNING=$(ps --no-headers -C roscore | wc -l)
    if [ "$RUNNING" -eq 1 ] ; then
	sleep 1
	roslaunch --pid=/home/smartlab/.ros/mitro_bringup.pid mitro_bringup mitro.launch
        DONE=1
    else
	sleep 1
	echo "no roscore started... waiting"
    fi
done
