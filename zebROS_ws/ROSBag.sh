#!/usr/bin/env bash

# Launch rqt_plot and input rosbag data from rosbag_rqt_graphing.launch data
source /opt/ros/kinetic/setup.bash
source ~/2018RobotCode/zebROS_ws/devel/setup.bash

roscore

rosrun rqt_plot

roslaunch controller_node rosbag_rqt_graphing.launch rosbag_data:=pembrooke_2018-03-16-18-38-47.bag.active data data1:= data2:= x-axis:= 

exec"$@"
