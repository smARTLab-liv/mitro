#!/bin/bash
PID_FILE="/home/smartlab/.ros/multicam.pid"
LOG_FILE="/home/smartlab/.ros/multicam.log"
rm -f $PID_FILE
rm -f $LOG_FILE
touch $LOG_FILE

source /opt/ros/indigo/setup.bash
source /home/smartlab/ros_ws/devel/setup.bash

DONE=0
while [ $DONE -eq 0 ]; do
    RUNNING=$(ps --no-headers -C roscore | wc -l)
    if [ "$RUNNING" -eq 1 ] ; then
	sleep 1
	/home/smartlab/ros_ws/devel/lib/mitro_multicam/multicam /dev/mitro/video_front /dev/mitro/video_bottom /dev/video10 > $LOG_FILE &
        sleep 1
	PID=`ps aux | grep 'mitro_multicam/multicam' | grep -v grep | grep -v ps3 | awk '{print $2}'`
	echo $PID > $PID_FILE
	echo "process started with PID: " $PID
	DONE=1
    else
	sleep 1
	echo "waiting until ros is started ..."
    fi
done
