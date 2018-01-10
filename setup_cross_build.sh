# script to install cross-development environment on a laptop
# This should let people build robot (roboRIO) code on a laptop

# Install toolchain 
sudo add-apt-repository ppa:wpilib/toolchain
sudo add-apt-repository ppa:wpilib/toolchain-beta
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install git libc6-i386 curl jstest-gtk gradle oracle-java8-installer frc-toolchain meshlab cmake libprotobuf-dev libprotoc-dev protobuf-compiler ninja-build sip-dev python-empy libtinyxml2-dev libeigen3-dev libpython2.7-dev

#mkdir -p ~/Downloads
#cd ~/Downloads
#wget http://mirror.csclub.uwaterloo.ca/eclipse/oomph/epp/neon/R/eclipse-inst-linux64.tar.gz
#tar -xzvf eclipse-inst-linux64.tar.gz
#cd eclipse-installer
#./eclipse-inst

# follow installer prompts : select C++ development environment when prompted

# start eclipse, install plugins : 

# Help -> Install new software -> Add
    # Name: FRC Plugins
    # Location: http://first.wpi.edu/FRC/roborio/release/eclipse/
    # Click OK
    # Expand the WPILib Robot Development repo and choose to install the Robot C++ Development plugin.
    # Follow/confirm the wizard/prompts


# Install CTRE libraries for TalonSRX controller
# included in above for now but keep handy
# for upgrading if needed
#
# Even more obsolete now - 2018 beta uses v5.0.x and seems to be
# installed automatically?
#cd
#wget http://www.ctr-electronics.com//downloads/lib/CTRE_FRCLibs_NON-WINDOWS_v4.4.1.14.zip
#mkdir ctre
#cd ctre
#unzip ../CTRE_FRCLibs_NON-WINDOWS_v4.4.1.14.zip
#cp -r cpp ~/wpilib/user
#cd ..
#rm -rf ctre CTRE_FRCLibs_NON-WINDOWS_v4.4.1.14.zip

# Get ros for RoboRIO libraries
cd
wget -O roscore_roborio_2018.tar.bz2 "https://drive.google.com/uc?export=download&id=1IggQt9DtTnZb6R1hxzKRPCUhEW44rQLj"
cd /usr/arm-frc-linux-gnueabi
sudo tar -xjf ~/roscore_roborio_2018.tar.bz2
cd
rm roscore_roborio_2018.tar.bz2

# Install roboRIO packages into the cross-root
sudo perl ~/2018RobotCode/install_cross_package.pl

# Clone 2018RobotCode repo
cd
git clone https://github.com/FRC900/2018RobotCode.git

# Build/install cross version of console_bridge
cd
git clone https://github.com/ros/console_bridge
cd console_bridge
git checkout 0.3.2
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
sudo make -j4 install
cd
sudo rm -rf console_bridge

# Build and install Poco libraries
cd
wget https://pocoproject.org/releases/poco-1.7.9/poco-1.7.9p1.tar.gz
tar xzf poco-1.7.9p1.tar.gz
cd poco-1.7.9p1/
CROSS_COMPILE=arm-frc-linux-gnueabi- ./configure --no-tests --no-samples --omit=Data/ODBC,Data/MySQL --minimal --prefix=/usr/arm-frc-linux-gnueabi/usr/local --static
sudo CROSS_COMPILE=arm-frc-linux-gnueabi- make -j4 install
cd
sudo rm -rf poco-1.7.9p1.tar.gz poco-1.7.9p1

#Hack in a cmake file for Eigen3
sudo mkdir -p /usr/arm-frc-linux-gnueabi/usr/lib/cmake/eigen3
sudo cp /usr/lib/cmake/eigen3/Eigen3Config.cmake /usr/arm-frc-linux-gnueabi/usr/lib/cmake/eigen3

# Build and instll SIP libraries
cd
wget https://downloads.sourceforge.net/project/pyqt/sip/sip-4.17/sip-4.17.tar.gz
tar -xzvf sip-4.17.tar.gz
cd sip-4.17
python configure.py CC=arm-frc-linux-gnueabi-gcc CXX=arm-frc-linux-gnueabi-g++ LINK=arm-frc-linux-gnueabi-g++ LINK_SHLIB=arm-frc-linux-gnueabi-g++ --sysroot=/usr/arm-frc-linux-gnueabi --incdir=/usr/arm-frc-linux-gnueabi/usr/include/python2.7 STRIP=arm-frc-linux-gnueabi-strip

#edit siplib/Makefile to change CPP flags include to -I/usr/arm-frc-linux-gnueabi/usr/include/python2.7
sed -i '/^CPPFLAGS/ s_include_usr/include_' siplib/Makefile
sudo make -j8 install
cd
sudo rm -rf sip-4.17.tar.gz sip-4.17


# Build and install tinyxml
wget https://github.com/leethomason/tinyxml2/archive/6.0.0.tar.gz
tar -xzvf 6.0.0.tar.gz
cd tinyxml2-6.0.0
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_STATIC_LIBS:BOOL=ON -DBUILD_TESTS:BOOL=OFF .
sudo make -j8 install
cd
rm -rf 6.0.0.tar.gz tinyxml2-6.0.0

# Different code uses different versions of tinyxml
cd
wget https://downloads.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.zip
unzip tinyxml_2_6_2.zip
cd tinyxml
wget https://gist.githubusercontent.com/TNick/7960323/raw/3046ecda1d4d54d777c407f43ac357846a192e05/TinyXML-CmakeLists.txt
mv TinyXML-CmakeLists.txt CMakeLists.txt
#add a line to CMakeLists.txt :  
sed -i "14i  set_target_properties(tinyxml PROPERTIES PUBLIC_HEADER \"tinyxml.h;tinystr.h\")" CMakeLists.txt
#add a line to tinyxml.h before line 46 :
sed -i '45i  #define TIXML_USE_STL' tinyxml.h
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
sudo make -j8 install
cd
sudo rm -rf tinyxml_2_6_2.zip tinyxml

# Build and install google flags library
cd
wget https://github.com/gflags/gflags/archive/v2.2.1.tar.gz
tar -xzvf v2.2.1.tar.gz
cd gflags-2.2.1
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi  -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
sudo make -j4 install
cd
sudo rm -rf v2.2.1.tar.gz gflags-2.2.1

# Build and install google logging libraries
cd
wget https://github.com/google/glog/archive/v0.3.5.tar.gz
tar -xzvf v0.3.5.tar.gz
cd glog-0.3.5/
CFLAGS="-O2 -fPIC -mcpu=cortex-a9 -mfpu=neon" CXXFLAGS="-O2 -fPIC -mcpu=cortex-a9 -mfpu=neon" LDFLAGS="-fPIC" ./configure --host=arm-frc-linux-gnueabi --prefix=/usr/arm-frc-linux-gnueabi/usr/local
sudo make -j8 install
cd
sudo rm -rf v0.3.5.tar.gz glog-0.3.5

# Build and install qhull libraries
cd
wget http://www.qhull.org/download/qhull-2015-src-7.2.0.tgz
tar -xzvf qhull-2015-src-7.2.0.tgz
cd qhull-2015.2/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi .
sudo make -j8 install
cd
sudo rm -rf qhull-2015-src-7.2.0.tgz qhull-2015.2

# Build and install assimp libraries
cd
git clone https://github.com/assimp/assimp.git
cd assimp
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi  ..
sudo make -j8 install
cd
sudo rm -rf assimp

# Build and install UUID libraries
cd
wget https://downloads.sourceforge.net/project/libuuid/libuuid-1.0.3.tar.gz
tar -xzvf libuuid-1.0.3.tar.gz
cd libuuid-1.0.3
CFLAGS="-O2" CXXFLAGS="-O2" ./configure --host=arm-frc-linux-gnueabi --prefix=/usr/arm-frc-linux-gnueabi/usr/local
sudo make install
cd
sudo rm -rf libuuid-1.0.3.tar.gz libuuid-1.0.3

#cd
#git clone https://github.com/rdiankov/collada-dom.git
#cd collada-dom
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/2018RobotCode/zebROS_ws/rostoolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi .
#sudo make -j8 install
#cd
#sudo rm -rf collada-dom

