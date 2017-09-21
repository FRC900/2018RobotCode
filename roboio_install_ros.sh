opkg update
opkg install boost-dev libeigen-dev libpython2 python-core python-dev libcurl4 lz4 libbz2-0 cmake-dev cmake libxml2-dev libxml2 libgnutls-bin libgnutls-dev libgnutls-openssl27 libgnutls28 libgnutlsxx28 nettle libgmp10 libgmpxx4 libz-dev libz1 git make gcc g++ gcc-symlinks g++-symlinks binutils python-setuptools python-docutils python-pyyaml python-pkgutil python-dateutil python-argparse python-nose python-netifaces libglog0 libglog-dev libyaml-dev python-pip coreutils

pip install catkin_pkg rospkg rosdistro vcstools rosdep wstool rosinstall rosinstall_generator defusedxml empy

cd
git clone https://github.com/ros/console_bridge
cd console_bridge
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make install
cd
rm -rf console_bridge

cd
git clone https://github.com/gflags/gflags.git
cd gflags
cmake -DCMAKE_BUILD_TYPE=Release  .
sudo make install
cd
rm -rf gflags*

# KCJ - I'm skeptical any of the below libs are really
# needed.  Many of the produce static libs so installed
# ROS components are already linked against them during
# the cross-build process.  Others are used by esoteric
# packages we'll probably never bother with.  If we end
# up getting a missing library error, though, the info
# on how to build them is here
cd
wget https://pocoproject.org/releases/poco-1.7.8/poco-1.7.8p3.tar.gz
tar xzf poco-1.7.8p3.tar.gz 
cd poco-1.7.8p3/
./configure --no-tests --no-samples --omit=Data/ODBC,Data/MySQL --minimal
#cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_MONGODB=OFF -DENABLE_CRYPTO=OFF -DENABLE_NET=OFF -DENABLE_NETSSL=OFF -DENABLE_DATA=OFF -DENABLE_ZIP=OFF -DENABLE_PAGECOMPILER=OFF -DENABLE_PAGECOMPILER_FILE2PAGE=OFF .
make -j2 install
cd
rm -rf poco-1.7.8p poco-1.7.8p3.tar.gz 

cd
wget https://downloads.sourceforge.net/project/pyqt/sip/sip-4.17/sip-4.17.tar.gz
tar -xzvf sip-4.17.tar.gz
cd sip-4.17
python configure.py
make -j2 install
cd
rm -rf sip-4.17*

# Probably not needed since it produces a static library
cd
wget https://downloads.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.zip
unzip tinyxml_2_6_2.zip 
cd tinyxml
wget https://gist.githubusercontent.com/TNick/7960323/raw/3046ecda1d4d54d777c407f43ac357846a192e05/TinyXML-CmakeLists.txt
mv TinyXML-CmakeLists.txt CMakeLists.txt
add a line to CMakeLists.txt :  
  set_target_properties(tinyxml PROPERTIES PUBLIC_HEADER "tinyxml.h;tinystr.h")
add a line to tinyxml.h before line 46 :
  #define TIXML_USE_STL
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
make -j2 install 
cd
rm -rf tinyxml*

cd
wget http://www.qhull.org/download/qhull-2015-src-7.2.0.tgz
tar -xzvf qhull-2015-src-7.2.0.tgz
cd qhull-2015.2/
cmake -DCMAKE_BUILD_TYPE=Release .
make -j2 install
cd
rm -rf qhull-2015*

cd
git clone https://github.com/assimp/assimp.git 
cd assimp
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .
make -j2 install
cd
rm -rf assimp


cd
wget https://downloads.sourceforge.net/project/libuuid/libuuid-1.0.3.tar.gz
tar -xzvf libuuid-1.0.3.tar.gz
cd libuuid-1.0.3/
./configure 
make -j2 install
cd
rm -rf libuuid-1.0.3*

