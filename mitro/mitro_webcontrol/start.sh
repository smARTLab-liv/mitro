#!/bin/bash
PID_FILE="/home/smartlab/.ros/webcontrol.pid"
LOG_FILE="/home/smartlab/.ros/webcontrol.log"
rm -f $PID_FILE
rm -f $LOG_FILE
touch $LOG_FILE

source /opt/ros/indigo/setup.bash
source /home/smartlab/ros_ws/devel/setup.bash
export DISPLAY=:0.0

DONE=0
while [ $DONE -eq 0 ]; do
    RUNNING=$(ps --no-headers -C roscore | wc -l)
    if [ "$RUNNING" -eq 1 ] ; then
	sleep 1
	cd /home/smartlab/ros_ws/src/mitro/mitro/mitro_webcontrol/scripts
	./webcontrol.py --host 10.30.129.131 --port 8080  > $LOG_FILE &
	PID=`ps aux | grep 'webcontrol.py' | grep -v grep | awk '{print $2}'`
	echo $PID > $PID_FILE
	echo "process started with PID: " $PID
	DONE=1
    else
	sleep 1
	echo "waiting until ros is started ..."
    fi
done
