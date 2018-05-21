#!/bin/bash

# Run me with IP address of Rio as argument

# Set time and date on Rio to match the system
# we're installing from.  This should be close enough
# to reality to get past ssl errors from pip if the 
# date on the Rio is totally wacky
ssh admin@$1 date --set="\"`date -R`\"" 

ssh admin@$1 'opkg update'

# Split these up so the disk doesn't fill up with temp files
# Also need to install pyyaml first for some reason to avoid
# weird dependency hell issues with opkg
ssh admin@$1 'opkg install python-pyyaml'
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install libeigen python-dev libpython2 python-core '
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install libcurl4 lz4 libboost-filesystem1.60.0 libboost-program-options1.60.0 libboost-signals1.60.0 libboost-regex1.60.0 libboost-thread1.60.0 libboost-chrono1.60.0 libboost-date-time1.60.0 libboost-atomic1.60.0'
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install libbz2 cmake libxml2 libgnutls-bin libgnutls-openssl27  '
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install libgnutls30 libgnutlsxx28 nettle libgmp10 libgmpxx4 libz1 git make '
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install gcc g++ gcc-symlinks g++-symlinks binutils python-setuptools python-docutils '
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install python-pkgutil python-dateutil python-argparse python-nose '
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install python-netifaces libglog0 python-pip coreutils gdb ntp i2c-tools '
ssh admin@$1 'opkg clean'
ssh admin@$1 'opkg install ntp ntp-tickadj ntp-utils ntpdate rsync htop curl'
ssh admin@$1 'opkg clean'

ssh admin@$1 'pip install catkin_pkg rospkg rosdistro vcstools rosdep wstool rosinstall rosinstall_generator defusedxml empy'

# Copy over ROS tar.bz2 file, extract to / on the Rio
wget -O /tmp/roscore_roborio_2018.tar.bz2 "https://drive.google.com/uc?export=download&id=1tT988R2pq9oyZyLWl1eDzMSpxcV7-HZ-"
scp /tmp/roscore_roborio_2018.tar.bz2 admin@$1:.
rm /tmp/roscore_roborio_2018.tar.bz2 
ssh admin@$1 'cd / && tar -xjf ~/roscore_roborio_2018.tar.bz2'
scp ~/2018RobotCode/os_detect.py admin@$1:/usr/lib/python2.7/site-packages/rospkg/
ssh admin@$1 'rm ~/roscore_roborio_2018.tar.bz2'

# Try to simulate what the cross-build environment
# looks like 
ssh admin@$1 'ln -s / /usr/arm-frc-linux-gnueabi'
ssh admin@$1 'ln -s /usr/include /include'

# Create workspace. Do a build in the empty workspace to set
# up various scripts for later use
ssh admin@$1 'mkdir -p 2018RobotCode/zebROS_ws/src'
ssh admin@$1 'source /opt/ros/kinetic/setup.bash && cd 2018RobotCode/zebROS_ws && catkin_make_isolated --install'

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
##################-----------------------------#################
#Edit /etc/ntp.conf to be a copy of ntp-server in 2018RobotCode#
scp ~/2018RobotCode/ntp-server.conf admin@$1:/etc/ntp.conf

#---------------------------------------------------------------#
#               to setup RTC on the rio
#   Plug the rtc into the i2c port on the rio while unpowered
#
scp ~/2018RobotCode/rtc-bq32k.ko admin@$1:.
ssh admin@$1 'mv rtc-bq32k.ko /lib/modules/`uname -r`/kernel'
ssh admin@$1 'depmod'
ssh admin@$1 'i2cdetect -y 2'
ssh admin@$1 'echo bq32000 0x68 | tee /sys/class/i2c-adapter/i2c-2/new_device'

ssh admin@$1 'git clone https://github.com/gflags/gflags.git'
ssh admin@$1 'cd gflags && cmake -DCMAKE_BUILD_TYPE=Release . &&  make install'
ssh admin@$1 'rm -rf gflags*'

# Copy wpilib to roborio
cd ~/wpilib/cpp/current/reflib/linux/athena/shared
scp *.so.* admin@$1:wpilib
cd ~/wpilib/common/current/lib/linux/athena/shared
scp *.so *.so.3.2 admin@$1:wpilib

scp ~/2018RobotCode/setupClock admin@$1:/etc/init.d
ssh admin@$1 'chmod +x /etc/init.d/setupClock'
ssh admin@$1 'ln -sf /etc/init.d/setupClock /etc/init.d/hwclock.sh'
ssh admin@$1 '/usr/sbin/update-rc.d -f setupClock defaults'
ssh admin@$1 '/usr/sbin/update-rc.d -f hwclock.sh defaults'

scp ~/2018RobotCode/jetson_setup/roborio_dot_ssh.tar.bz2 admin@$1:.
ssh admin@$1 'mkdir .ssh'
ssh admin@$1 'cd .ssh && tar -xjf ../roborio_dot_ssh.tar.bz2'
ssh admin@$1 'rm roborio_dot_ssh.tar.bz2'

# Edit /etc/ssh/sshd_config, uncomment Port 22, add Port 5801, 
# uncomment ChallengeResponseAuthentication and set it to no
ssh admin@$1 "sed \"s/#Port 22/Port 22\\nPort 5801/g\" /etc/ssh/sshd_config | sed \"s/#ChallengeResponseAuthentication yes/ChallengeResponseAuthentication no/\" > sshd_config && mv sshd_config /etc/ssh"

# Copy rio_bashrc.sh, ROSJetsonMaster.sh to /home/admin
scp ~/2018RobotCode/rio_bashrc.sh admin@$1:.
scp ~/2018RobotCode/zebROS_ws/ROSJetsonMaster.sh admin@$1:.

# Set up prereqs for deploy script
ssh admin@$1 'mv ~/2018RobotCode ~/2018RobotCode.orig'
ssh admin@$1 'ln -s ~/2018RobotCode.orig ~/2018RobotCode'
ssh admin@$1 'mkdir -p ~/2018RobotCode.prod/zebROS_ws'
ssh admin@$1 'mkdir -p ~/2018RobotCode.dev/zebROS_ws'
