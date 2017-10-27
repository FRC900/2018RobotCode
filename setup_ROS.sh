#!/bin/bash
# ROS Setup install script for new jetsons
# Source: https://github.com/jetsonhacks/installROSTX1/blob/master/installROS.sh

# Run me after running through setup_environment.sh
# TODO - any reason not to merge these files?

# Setup Locale
# sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
# Installation
sudo apt update
sudo apt install ros-kinetic-ros-base -y

# Add Individual Packages here
# You can install a specific ROS package (replace underscores with dashes of the package name):
# sudo apt-get install ros-kinetic-PACKAGE
# e.g.
# sudo apt-get install ros-kinetic-navigation
#
# To find available packages:
# apt-cache search ros-kinetic
# 
sudo apt install python-rosdep python-rosinstall terminator ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-tf2-ros ros-kinetic-pcl-conversions ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-map-server ros-kinetic-rviz ros-kinetic-hector-slam ros-kinetic-hector-slam-launch ros-kinetic-rtabmap-ros ros-kinetic-robot-localization ros-kinetic-navigation ros-kinetic-robot-state-publisher ros-kinetic-rosparam-shortcuts python-wstool ninja-build libsuitesparse-dev ros-kinetic-tf2-tools ros-kinetic-hardware-interface ros-kinetic-controller-manager ros-kinetic-control-msgs ros-kinetic-joint-limits-interface ros-kinetic-transmission-interface ros-kinetic-control-toolbox liblua5.3-dev ros-kinetic-joystick-drivers ros-kinetic-gmapping ros-kinetic-teb-local-planner ros-kinetic-roslint ros-kinetic-xacro ros-kinetic-rqt-controller-manager ros-kinetic-ros-controllers

# Initialize rosdep
# ssl certificates can get messed up on TX1 for some reason
sudo c_rehash /etc/ssl/certs
# Initialize rosdep
sudo rosdep init
# To find available packages, use:
rosdep update

cd ~/2017Preseason/zebROS_ws/src
#wstool init
#cd ..

# These should all be merged in the GIT repo version
# of the code.  Check 2017Preseason/zebROS_ws/src/.rosinstall to
# verify.  Leaving the commands here just in case we need to recreate
# a workspace elsewhere

# Google Cartographer installation forked from https://google-cartographer-ros.readthedocs.io/en/latest/
# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
#wstool merge https://raw.githubusercontent.com/FRC900/cartographer_ros/master/cartographer_ros.rosinstall

# Code to use joystick to control robot
#wstool merge https://raw.githubusercontent.com/FRC900/teleop_twist_joy/indigo-devel/teleop_twist_joy.rosinstall
 
# Merge ZED wrapper
#wstool merge https://raw.githubusercontent.com/FRC900/zed-ros-wrapper/master/zed_ros_wrapper.rosinstall

# Boilerplate control code
#wstool merge https://raw.githubusercontent.com/FRC900/steered_wheel_base_controller/master/steered_wheel_base_controller.rosinstall
#wstool merge https://raw.githubusercontent.com/FRC900/ros_control_boilerplate/kinetic-devel/ros_control_boilerplate.rosinstall

# LIDAR driver
#wstool merge https://raw.githubusercontent.com/FRC900/rplidar_ros/master/rplidar.rosinstall

# Pull latest version of merged repos 
wstool update

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro=kinetic -y

source /opt/ros/kinetic/setup.bash

cd ~/2017Preseason/zebROS_ws/src
catkin_make
