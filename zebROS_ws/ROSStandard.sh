#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/kinetic/setup.bash
source ~/2018RobotCode/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROS_IP=`/bin/hostname -I | tr -d ' ' | tr -d '\n'`
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
