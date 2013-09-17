#!/bin/bash
PID_FILE="/home/swarmlab/multicam.pid"
LOG_FILE="/home/swarmlab/multicam.log"
rm -f $PID_FILE
rm -f $LOG_FILE
touch $LOG_FILE

source /opt/ros/hydro/setup.bash
source /home/swarmlab/ros_catkin_ws/devel/setup.bash

DONE=0
while [ $DONE -lt 1 ]; do
    if rostopic list > /dev/null
    then
	sleep 1
	/home/swarmlab/ros_catkin_ws/src/mitro/mitro_multicam/bin/multicam /dev/video1 /dev/video2 /dev/video3 > $LOG_FILE &
        sleep 1
	PID=`ps aux | grep 'bin/multicam' | grep -v grep | grep -v ps3 | awk '{print $2}'`
	echo $PID > $PID_FILE
	echo "process started with PID: " $PID
	DONE=1
    else
	sleep 1
	echo "waiting until ros is started ..."
    fi
done

