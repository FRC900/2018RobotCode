#!/bin/bash

# Reload IPv6 networking blocks
sudo sysctl -p

. /home/ubuntu/2018RobotCode/zebROS_ws/ROSJetsonMaster.sh
#echo 1100-1200,443,80,554,1735 > /proc/sys/net/ipv4/ip_local_reserved_ports

#echo 5800 5810 > /proc/sys/net/ipv4/ip_local_port_range
#systemctl restart networking

sudo chmod a+rw /dev/ttyACM0
#sudo python /home/ubuntu/2017VisionCode/time_sync_server.py & 
sudo umount /mnt/900_2

export CUDA_CACHE_MAXSIZE=104857600
export CUDA_CACHE_PATH=/home/ubuntu/.nv/ComputeCache

if sudo mount /dev/disk/by-id/$(ls /dev/disk/by-id/ | grep 'SanDisk.*part1') /mnt/900_2; then
		sudo chmod a+rw /mnt/900_2/
		roslaunch controller_node controller_master.launch record:=true
else
		roslaunch controller_node controller_master.launch
fi

nvpmodel -m 0
/home/ubuntu/jetson_clocks.sh
/home/ubuntu/2018RobotCode/jetson_setup/clocks.sh &

