#!/bin/bash
jetson=true
version="tx2"
gpu=true

#process args
while [ $# -gt 0 ]
do
	case "$1" in
	-tx1) jetson=true;verson="tx1";;
	-tx2) jetson=true;version="tx2";;
	-amd64) jetson=false;;
	-c) gpu=false;;
	-h) echo >&2 \
		"usage: $0 [-tx1 or -tx2 or -amd64] [-c] [-h]"
		exit 1;;
	*)  break;;	# terminate while loop
	esac
	shift
done

#install basic dependencies

sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get install -y libeigen3-dev build-essential gfortran git cmake libleveldb-dev libsnappy-dev libhdf5-dev libhdf5-serial-dev liblmdb-dev vim-gtk libgflags-dev libgoogle-glog-dev libatlas-base-dev python-dev python-pip libtinyxml2-dev v4l-conf v4l-utils libgtk2.0-dev pkg-config exfat-fuse exfat-utils libprotobuf-dev protobuf-compiler unzip python-numpy python-scipy python-opencv python-matplotlib chromium-browser wget unzip ccache

sudo apt-get install --no-install-recommends -y libboost-all-dev

#install caffe
# cd
# git clone https://github.com/BVLC/caffe.git
# cd caffe
# mkdir build
# cd build

# if [ "$gpu" == "false" ] ; then
	# cmake -DCPU_ONLY=ON ..
# else
	# cmake -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF ..
# fi

# make -j4 all
#make test
#make runtest
# make -j4 install

# Install libsodium - this is a prereq for zeromq 1.0.14 released 9/21/2017
cd
wget --no-check-certificate https://download.libsodium.org/libsodium/releases/libsodium-1.0.14.tar.gz
tar -zxvf libsodium-1.0.14.tar.gz
cd libsodium-1.0.14
./configure
make -j4 
sudo make install
cd ..
rm -rf libsodium-1.0.14*

# install zeromq 4.2.2 is latest stable as of 9/20/2017
cd
wget --no-check-certificate https://github.com/zeromq/libzmq/releases/download/v4.2.2/zeromq-4.2.2.tar.gz
tar -xzvf zeromq-4.2.2.tar.gz
cd zeromq-4.2.2
./configure
make -j4
sudo make install
cd ..
rm -rf zeromq-4.2.2*
cd /usr/local/include/
sudo wget --no-check-certificate https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq.hpp

# Install tinyxml2
cd
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
rm -rf tinyxml2

# Install Point Cloud Library
sudo apt-get install libflann-dev libpcl-dev
#cd
#wget --no-check-certificate https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.zip
#unzip pcl-1.8.0.zip
#cd pcl-pcl-1.8.0
#mkdir build
#cd build
#cmake ..
#make -j4
#sudo make install
#cd ../..
#rm -rf pcl-1.8.0.zip pcl-pcl-1.8.0

#install zed sdk
if [ "$version" = tx1 ] && [ "$jetson" = true ] ; then
	$zed_arch="JTX1"
elif [ "$version" = tx2 ] && [ "$jetson" = true ] ; then
	$zed_arch="JTX2"
else
	$zed_arch="Ubuntu16"
fi

#https://www.stereolabs.com/developers/downloads/ZED_SDK_Linux_JTX1_v2.1.2.run
#https://www.stereolabs.com/developers/downloads/ZED_SDK_Linux_JTX1_v2.1.2.run
#https://www.stereolabs.com/developers/downloads/ZED_SDK_Linux_Ubuntu16_v2.1.2.run

zed_ver="2.3.0"
zed_fn="ZED_SDK_Linux_"$zed_arch"_v"$zed_ver".run"
wget --no-check-certificate https://www.stereolabs.com/download/$zed_fn
chmod 755 $zed_fn
./$zed_fn
rm ./$zed_fn


#clone repo
#TODO : rethink this - how are we getting the script if the
#       repo isn't there in the first place?
cd
#git clone https://github.com/FRC900/2018RobotCode.git
cd 2018RobotCode
git submodule init
git submodule update

#mount and setup autostart script
if [ "$jetson" = true ] ; then
	sudo mkdir /mnt/900_2

	# For tx2 only - install drivers for USB
	# serial devices
	if [ "$version" = tx2 ] ; then
		sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
		sudo cp jetson_setup/cp210x.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x.ko
		sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
		sudo cp jetson_setup/cdc-acm.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
		sudo depmod -a
	fi

	# Kernel module build steps for TX2 : https://gist.github.com/sauhaardac/9d7a82c23e4b283a1e79009903095655
	# Not needed unless Jetpack is updated and modules
	# for a given kernel version aren't already built
	# cd ~
	# mkdir l4t-kernel-surgery
	# mkdir kernel-stuff
	# cd kernel-stuff
	# wget http://developer.nvidia.com/embedded/dlc/l4t-sources-28-1 -O source.tbz2
	# tar xjf source.tbz2
	# tar xjf sources/kernel_src-tx2.tbz2 -C ~/l4t-kernel-surgery/
	# cd ..
	# rm -rf kernel-stuff
	# cd ~/l4t-kernel-surgery/kernel/kernel-4.4
	# Add EXTRAVERSION=-tegra to Makefile
	# zcat /proc/config.gz > .config
	# echo "CONFIG_USB_ACM=m" >> .config
	# echo "CONFIG_USB_SERIAL_CP210X=m" >> .config
	# make -j6 clean
	# make -j6 prepare
	# make -j6 modules_prepare
	# make -j6 M=drivers/usb/class
	# make -j6 M=drivers/usb/serial
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
	# sudo cp drivers/usb/class/cp210x-acm.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x-acm.ko
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
	# sudo cp drivers/usb/serial/cdc-acm.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/joystick
	# sudo cp xpad.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/joystick/xpad.ko
	# sudo depmod -a
fi

sudo mkdir -p /usr/local/zed/settings
sudo chmod 755 /usr/local/zed/settings
sudo cp ~/2018RobotCode/calibration_files/*.conf /usr/local/zed/settings
sudo chmod 644 /usr/local/zed/settings/*

cp ~/2018RobotCode/.vimrc ~/2018RobotCode/.gvimrc ~
sudo cp ~/2018RobotCode/kjaget.vim /usr/share/vim/vim74/colors

git config --global user.email "progammers@team900.org"
git config --global user.name "Team900 Jetson TX2"

sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.gold" 20
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.bfd" 10

sudo update-alternatives --config ld
