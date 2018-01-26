#!/usr/bin/env bash

# Setup Rio ROS Environment
source /opt/ros/kinetic/setup.bash
source ~/2018RobotCode/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.9.0.2
export ROS_IP=$(hostname -I)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH


echo \#!/bin/bash > setupClock
echo NAME="Setup Clock" >> setupClock
echo USER=admin >> setupClock
echo depmod echo bq32000 0x68 | tee /sys/class/i2c-adapter/i2c-2/new_device >> setupClock
echo i2cdetect -y 2 >> setupClock
echo hwclock.util-linux --hctosys >> setupClock

chmod +x setupClock
cp setupClock /etc/init.d
/usr/sbin/update-rc.d -f setupClock defaults






exec "$@"
