#!/usr/bin/env bash

# Setup Rio ROS Environment
source /opt/ros/kinetic/setup.bash
source ~/2017VisionCode/zebROS_ws/install_isolated/setup.bash
export ROS_MASTER_URI=http://10.0.20.74:5802
export ROS_IP=$(hostname -I)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
