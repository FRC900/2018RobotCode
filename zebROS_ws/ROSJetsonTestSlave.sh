#!/usr/bin/env bash

# Setup ROS for Jetson Slave
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/2017VisionCode/zebROS_ws/install_isolated/setup.bash
export ROS_MASTER_URI=http://10.9.0.12:5802
export ROS_IP=$(hostname -I)
export ROSLAUNCH_SSH_UNKNOWN=1
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
