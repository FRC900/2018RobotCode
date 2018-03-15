#!/usr/bin/env bash

# Setup ROS for Jetson Master
if [ -f /home/ubuntu/2018RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Jetson environment"
    source /opt/ros/kinetic/setup.bash
    source /home/ubuntu/2018RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.8
elif [ -f /home/admin/rio_bashrc.sh ] ; then
    # roboRIO-specific configuration
    echo "Sourcing roboRIO environment"
    source /home/admin/rio_bashrc.sh
    export ROS_IP=10.9.0.2
    export LD_LIBRARY_PATH=/home/admin/wpilib:$LD_LIBRARY_PATH
    swapon /dev/sda5
else
    echo "Unknown environment! Trying to proceed anyway..."
fi

# Common configuration
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ROS_MASTER_URI=http://10.9.0.8:5802
#export ROS_IP=`/bin/hostname -I | tr -d ' ' | tr -d '\n'`
export ROSLAUNCH_SSH_UNKNOWN=1

exec "$@"
