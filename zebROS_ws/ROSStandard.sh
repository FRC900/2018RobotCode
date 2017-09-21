#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/kinetic/setup.bash
source ~/2017VisionCode/zebROS_ws/install_isolated/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROS_IP=$(hostname -I)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
