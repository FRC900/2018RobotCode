#!/usr/bin/env bash
# Launch rqt_plot and input rosbag data from rosbag_rqt_graphing.launch data
source /opt/ros/kinetic/setup.bash
source ~/2018RobotCode/zebROS_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311

#sim time (x-axis problem fix)
rosparam set use_sim_time true

#open roscore session
roscore&
sleep 4

######## data output ########
#data output if $2 -eq 0
#if [$2 -eq 0] then
rqt_plot /frcrobot/$2 /frcrobot/$3 /frcrobot/$4 /frcrobot/$5 /frcrobot/$6 /frcrobot/$7 /frcrobot/$8 /frcrobot/$9&

#data output if $2 != 0
#if [$2 -eq 1]
#for i in {3..10}
#do
#rqt_plot /frcrobot/$i
#done
#fi

######## rosbag input ########
#play the rosbag to link topic(s)
cd
gnome-terminal -- rosbag play --clock Downloads/$1
rqt_bag Downloads/$1
cd
sleep 3
