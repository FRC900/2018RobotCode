#!/bin/bash

source /opt/ros/kinetic/setup.bash
rosnode kill -a
killall -9 rosmaster
sleep 5
