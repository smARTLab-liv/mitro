#!/bin/bash
PID_FILE="/home/swarmlab/webcontrol.pid"
LOG_FILE="/home/swarmlab/webcontrol.log"
rm -f $PID_FILE
rm -f $LOG_FILE

source /home/swarmlab/.bashrc
export DISPLAY=:0.0

sleep 1
cd /home/swarmlab/ros/mitro/mitro_webcontrol/src
./webcontrol.py --host 10.10.14.2 --port 8080 &
ps aux | grep webcontrol.py | grep -v grep | awk '{print $2}' >> $PID_FILE

