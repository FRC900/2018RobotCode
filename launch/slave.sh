#!/bin/bash

# Reload IPv6 networking blocks
sudo sysctl -p

. /home/ubuntu/2017VisionCode/zebROS_ws/ROSJetsonSlave.sh
#echo 1100-1200,443,80,554,1735,5800-5810 > /proc/sys/net/ipv4/ip_local_reserved_ports
#sudo chmod a+rw /dev/ttyACM0

sudo umount /mnt/900_2
sudo python /home/ubuntu/2017VisionCode/time_sync_server.py &

export CUDA_CACHE_MAXSIZE=104857600
export CUDA_CACHE_PATH=/home/ubuntu/.nv/ComputeCache

if sudo mount /dev/disk/by-id/$(ls /dev/disk/by-id/ | grep 'SanDisk.*part1') /mnt/900_2; then
	sudo chmod a+rw /mnt/900_2/
	roslaunch controller_node controller_slave.launch record:=true
else
	roslaunch controller_node controller_slave.launch
fi

/home/ubuntu/2017VisionCode/launch/clocks.sh &

