#!/bin/bash
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=/home/swarmlab/ros:$ROS_PACKAGE_PATH
roslaunch --pid=/home/swarmlab/mitro_bringup.pid mitro_bringup minimal.launch &
sleep 8 && roslaunch --pid=/home/swarmlab/mitro_bob.pid mitro_bringup bnaic_bob.launch



