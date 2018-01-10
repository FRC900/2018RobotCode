#sed -i 's#nickdanger\.amer\.corp\.natinst\.com/feeds/\([^/]\+\)/\([^/]\+\)/#download.ni.com/ni-linux-rt/feeds/\1/\2/ipk/#' /etc/opkg/base-feeds.conf
opkg update

# Split these up so the disk doesn't fill up with temp files
# Also need to install pyyaml first for some reason to avoid
# weird dependency hell issues with opkg
opkg install python-pyyaml
opkg clean
opkg install libeigen python-dev libpython2 python-core 
opkg clean
opkg install libcurl4 lz4 libboost-filesystem1.60.0 libboost-program-options1.60.0 libboost-signals1.60.0 libboost-regex1.60.0 libboost-thread1.60.0 libboost-chrono1.60.0 libboost-date-time1.60.0 libboost-atomic1.60.0
opkg clean
opkg install libbz2 cmake libxml2 libgnutls-bin libgnutls-openssl27  
opkg clean
opkg install libgnutls30 libgnutlsxx28 nettle libgmp10 libgmpxx4 libz1 git make 
opkg clean
opkg install gcc g++ gcc-symlinks g++-symlinks binutils python-setuptools python-docutils 
opkg clean
opkg install python-pkgutil python-dateutil python-argparse python-nose 
opkg clean
opkg install python-netifaces libglog0 python-pip coreutils gdb
opkg clean

pip install catkin_pkg rospkg rosdistro vcstools rosdep wstool rosinstall rosinstall_generator defusedxml empy

# Copy over ROS tar.bz2 file, extract to /
source /opt/ros/kinetic/setup.bash

# Try to simulate what the cross-build environment
# looks like 
ln -s / /usr/arm-frc-linux-gnueabi
ln -s /usr/include /include

cd
mkdir -p 2018RobotCode/zebROS_ws/src
cd 2018RobotCode/zebROS_ws
catkin_make_isolated --install
cd 

cd
git clone https://github.com/gflags/gflags.git
cd gflags
cmake -DCMAKE_BUILD_TYPE=Release .
make install
cd
rm -rf gflags*

# Changed this to static lib on host, shouldn't need
# to be installed on target as well
#cd
#git clone https://github.com/ros/console_bridge
#cd console_bridge
#mkdir build
#cd build
#cmake -DCMAKE_BUILD_TYPE=Release ..
#make install
#cd
#rm -rf console_bridge

# Changed to static libs on host, shouldn't be needed on target

# This is pre-built on a flash drive - make install should just 
# check everything is up to date and then copy the libraries and headers
# over ... at least until the Rio image changes?
#cd
#wget https://pocoproject.org/releases/poco-1.7.9/poco-1.7.9p1.tar.gz
#tar -xzf poco-1.7.9p1.tar.gz 
#cd poco-1.7.9p1/
#./configure --no-tests --no-samples --omit=Data/ODBC,Data/MySQL --minimal
#cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_MONGODB=OFF -DENABLE_CRYPTO=OFF -DENABLE_NET=OFF -DENABLE_NETSSL=OFF -DENABLE_DATA=OFF -DENABLE_ZIP=OFF -DENABLE_PAGECOMPILER=OFF -DENABLE_PAGECOMPILER_FILE2PAGE=OFF .
#make -j2 install
#cd
#rm -rf poco-1.7.9p1 poco-1.7.9p1.tar.gz 

#scp ~/2018RobotCode/os_detect.py admin@<target>:/usr/lib/python2.7/site-packages/rospkg/

# Copy wpilib to roborio
# cd ~/wpilib/cpp/current/reflib/linux/athena/shared
#scp *.so.* <target>
#cd ~/wpilib/common/current/lib/linux/athena/shared
#scp *.so *.so.3.2 <target>

# KCJ - I'm skeptical any of the below libs are really
# needed.  Many of the produce static libs so installed
# ROS components are already linked against them during
# the cross-build process.  Others are used by esoteric
# packages we'll probably never bother with.  If we end
# up getting a missing library error, though, the info
# on how to build them is here
# cd
# wget https://downloads.sourceforge.net/project/pyqt/sip/sip-4.17/sip-4.17.tar.gz
# tar -xzvf sip-4.17.tar.gz
# cd sip-4.17
# python configure.py
# make -j2 install
# cd
# rm -rf sip-4.17*

# Probably not needed since it produces a static library
# cd
# wget https://downloads.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.zip
# unzip tinyxml_2_6_2.zip 
# cd tinyxml
# wget https://gist.githubusercontent.com/TNick/7960323/raw/3046ecda1d4d54d777c407f43ac357846a192e05/TinyXML-CmakeLists.txt
# mv TinyXML-CmakeLists.txt CMakeLists.txt
# add a line to CMakeLists.txt :  
  # set_target_properties(tinyxml PROPERTIES PUBLIC_HEADER "tinyxml.h;tinystr.h")
# add a line to tinyxml.h before line 46 :
  # #define TIXML_USE_STL
# cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
# make -j2 install 
# cd
# rm -rf tinyxml*

# cd
# wget http://www.qhull.org/download/qhull-2015-src-7.2.0.tgz
# tar -xzvf qhull-2015-src-7.2.0.tgz
# cd qhull-2015.2/
# cmake -DCMAKE_BUILD_TYPE=Release .
# make -j2 install
# cd
# rm -rf qhull-2015*

# cd
# git clone https://github.com/assimp/assimp.git 
# cd assimp
# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release .
# make -j2 install
# cd
# rm -rf assimp


# cd
# wget https://downloads.sourceforge.net/project/libuuid/libuuid-1.0.3.tar.gz
# tar -xzvf libuuid-1.0.3.tar.gz
# cd libuuid-1.0.3/
# ./configure 
# make -j2 install
# cd
# rm -rf libuuid-1.0.3*

