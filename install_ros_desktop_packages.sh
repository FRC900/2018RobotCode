#!/bin/bash

set -e
set -o pipefail

# List required ROS packages for desktop setup here:
DEBIAN_FRONTEND=noninteractive sudo apt update
DEBIAN_FRONTEND=noninteractive sudo apt install -y \
	ros-kinetic-desktop-full \
	python-rosinstall \
	python-rosinstall-generator \
	python-wstool \
	ros-kinetic-hardware-interface \
	ros-kinetic-realtime-tools \
	ros-kinetic-controller-interface \
	ros-kinetic-controller-manager \
	ros-kinetic-joint-limits-interface \
	ros-kinetic-transmission-interface \
	ros-kinetic-control-toolbox \
	ros-kinetic-rosparam-shortcuts \
	ros-kinetic-joint-trajectory-controller \
	ros-kinetic-serial \
	ros-kinetic-forward-command-controller \
	ntp

