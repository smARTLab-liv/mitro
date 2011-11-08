#!/bin/bash
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH=/home/swarmlab/ros:$ROS_PACKAGE_PATH
roslaunch --pid=/home/swarmlab/mitro_bringup.pid mitro_bringup minimal.launch
