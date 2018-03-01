#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/admin/2018RobotCode/zebROS_ws/install_isolated/setup.bash 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/admin/wpilib:/usr/local/lib
export ROS_MASTER_URI=http://10.9.0.8:5802
export ROS_IP=10.9.0.2
