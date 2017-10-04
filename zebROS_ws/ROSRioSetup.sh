#!/usr/bin/env bash

# Setup Rio ROS Environment
source /opt/ros/kinetic/setup.bash
source ~/2017Preseason/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.0.20.74:5802
export ROS_IP=$(hostname -I)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
