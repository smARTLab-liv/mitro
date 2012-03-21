#!/bin/bash
PID_FILE="/home/swarmlab/webcontrol.pid"
LOG_FILE="/home/swarmlab/webcontrol.log"
rm -f $PID_FILE
rm -f $LOG_FILE

source /opt/ros/electric/setup.bash
export ROS_MASTER_URI=http://bob:11311
export DISPLAY=:0.0

/home/swarmlab/ros/mitro_client/webcontrol/src/webcontrol.py --host 10.10.14.2 --port 8080 &> $LOG_FILE &

ps aux | grep webcontrol.py | grep -v grep | awk '{print $2}' >> $PID_FILE


