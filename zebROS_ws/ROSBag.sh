#!/usr/bin/env bash

# Launch rqt_plot and input rosbag data from rosbag_rqt_graphing.launch data
source /opt/ros/kinetic/setup.bash
source ~/2018RobotCode/zebROS_ws/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311

sleep 3

#open roscore session
roscore&

sleep 5
#data output
rostopic echo /frcrobot/pdp_states/temperature&
rqt_plot /frcrobot/pdp_states/temperature&

#play the rosbag to link topic(s)
cd
gnome-terminal -- rosbag play --clock Downloads/pembrooke_2018-03-16-18-38-47.bag.active
