#!/bin/bash
PID_FILE="/home/swarmlab/multicam.pid"
LOG_FILE="/home/swarmlab/multicam.log"
rm -f $PID_FILE
rm -f $LOG_FILE
touch $LOG_FILE

source /opt/ros/electric/setup.bash
export ROS_MASTER_URI=http://bob:11311

DONE=0
while [ $DONE -lt 1 ]; do
    if rostopic list > /dev/null
    then
	sleep 1
	/home/swarmlab/ros/mitro/mitro_multicam/bin/multicam /dev/video1 /dev/video2 /dev/video3 > $LOG_FILE &
	PID=`ps aux | grep multicam | grep -v grep | grep -v ps3 | awk '{print $2}'`
	echo $PID > $PID_FILE
	echo "process started with PID: " $PID
	DONE=1
    else
	sleep 1
	echo "waiting until ros is started ..."
    fi
done
