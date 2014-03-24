#!/bin/bash
PID_FILE="/home/swarmlab/.ros/webcontrol.pid"
LOG_FILE="/home/swarmlab/.ros/webcontrol.log"
rm -f $PID_FILE
rm -f $LOG_FILE

source /home/swarmlab/.bashrc
export DISPLAY=:0.0

sleep 1
cd /home/swarmlab/ros_catkin_ws/src/mitro/mitro_webcontrol/scripts
./webcontrol.py --host 10.10.16.1 --port 8080 &
ps aux | grep webcontrol.py | grep -v grep | awk '{print $2}' >> $PID_FILE

