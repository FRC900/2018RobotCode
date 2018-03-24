#!/usr/bin/env bash

# Launch rqt_plot and input rosbag data from rosbag_rqt_graphing.launch data
source /opt/ros/kinetic/setup.bash
source ~/2018RobotCode/zebROS_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311

#open roscore session
roscore&

sleep 4
#data output
rqt_plot /frcrobot/$2 /frcrobot/$3 /frcrobot/$4 /frcrobot/$5 /frcrobot/$6 /frcrobot/$7 /frcrobot/$8 /frcrobot/$9 /frcrobot/$10&

#play the rosbag to link topic(s)
cd
gnome-terminal -- rosbag play --clock Downloads/$1
sleep 15
gnome-terminal -- rostopic echo $2
gnome-terminal -- rostopic echo $3
gnome-terminal -- rostopic echo $4
gnome-terminal -- rostopic echo $5
