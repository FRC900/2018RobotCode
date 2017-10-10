#!/bin/bash

#Can only run as root

if [[ $EUID -ne 0 ]]; then
	echo "This script must be run as root" 1>&2
	exit 1
fi

#Check if master / slave and copy file accordingly

while [ $# -gt 0 ]
do
	case "$1" in
			-master) cp master_launch.service  /etc/systemd/system/ros_launch.service;;
		-slave) cp slave_launch.service /etc/systemd/system/ros_launch.service;;
		*) break ;;
	esac
	shift
done

#Set file permissions and reload systemd

chmod 664 /etc/systemd/system/ros_launch.service

sudo systemctl daemon-reload
sudo systemctl enable ros_launch 

echo "This Jetson will now run ROS $1 on boot. Run systemctl disable ros_launch.service to disable"
