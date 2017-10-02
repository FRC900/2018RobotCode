# script to install cross-development environment on a laptop
# This should let people build robot (roboRIO) code on a laptop

# Install toolchain 
sudo apt-add-repository ppa:wpilib/toolchain
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install git libc6-i386 curl jstest-gtk gradle oracle-java8-installer frc-toolchain meshlab cmake libprotobuf-dev libprotoc-dev protobuf-compiler

mkdir -p ~/Downloads
cd ~/Downloads
wget https://www.eclipse.org/downloads/download.php?file=/oomph/epp/neon/R/eclipse-inst-linux64.tar.gz&mirror_id=492 
tar -xzvf eclipse-inst-linux64.tar.gz 
cd eclipse-installer
./eclipse-inst

# follow installer prompts : select C++ development environment when prompted

# start eclipse, install plugins : 

# Help -> Install new software -> Add
    # Name: FRC Plugins
    # Location: http://first.wpi.edu/FRC/roborio/release/eclipse/
    # Click OK
    # Expand the WPILib Robot Development repo and choose to install the Robot C++ Development plugin.
    # Follow/confirm the wizard/prompts

# Install CTRE libraries for TalonSRX controller
cd
wget http://www.ctr-electronics.com//downloads/lib/CTRE_FRCLibs_NON-WINDOWS_v5.0.3.3.zip
mkdir ctre
cd ctre
unzip ../CTRE_FRCLibs_NON-WINDOWS_v5.0.3.3.zip
cp -r cpp ~/wpilib/user
cd ..
rm -rf ctre CTRE_FRCLibs_NON-WINDOWS_v5.0.3.3.zip

# Install roboRIO packages into the cross-root
perl install_cross_packages.pl

# Build/install cross version of console_bridge
cd
git clone https://github.com/ros/console_bridge
cd console_bridge
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`/usr/bin/frc-cmake-toolchain` -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi ..

# Build and install Poco libraries
cd
wget https://pocoproject.org/releases/poco-1.7.8/poco-1.7.8p3.tar.gz
tar xzf poco-1.7.8p3.tar.gz 
cd poco-1.7.8p3/
CROSS_COMPILE=arm-frc-linux-gnueabi- ./configure --no-tests --no-samples --omit=Data/ODBC,Data/MySQL --minimal --prefix=/usr/arm-frc-linux-gnueabi/usr/local
CROSS_COMPILE=arm-frc-linux-gnueabi- make -j8 
cd
rm -rf poco-1.7.8p3.tar.gz poco-1.7.8p3


# Build and instll SIP libraries
cd
wget https://downloads.sourceforge.net/project/pyqt/sip/sip-4.17/sip-4.17.tar.gz
tar -xzvf sip-4.17.tar.gz
cd sip-4.17
python configure.py CC=arm-frc-linux-gnueabi-gcc CXX=arm-frc-linux-gnueabi-g++ LINK=arm-frc-linux-gnueabi-g++ LINK_SHLIB=arm-frc-linux-gnueabi-g++ --sysroot=/usr/arm-frc-linux-gnueabi --incdir=/usr/arm-frc-linux-gnueabi/usr/include/python2.7 STRIP=arm-frc-linux-gnueabi-strip

edit siplib/Makefile to change CPP flags include to -I/usr/arm-frc-linux-gnueabi/usr/include/python2.7
sudo make install
cd
rm -rf sip-4.17.tar.gz sip-4.17


# Build and install tinyxml
cd
wget https://downloads.sourceforge.net/project/tinyxml/tinyxml/2.6.2/tinyxml_2_6_2.zip
unzip tinyxml_2_6_2.zip 
cd tinyxml
wget https://gist.githubusercontent.com/TNick/7960323/raw/3046ecda1d4d54d777c407f43ac357846a192e05/TinyXML-CmakeLists.txt
mv TinyXML-CmakeLists.txt CMakeLists.txt
#add a line to CMakeLists.txt :  
#  set_target_properties(tinyxml PROPERTIES PUBLIC_HEADER "tinyxml.h;tinystr.h")
#add a line to tinyxml.h before line 46 :
#  #define TIXML_USE_STL
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`/usr/bin/frc-cmake-toolchain` -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi -DCMAKE_POSITION_INDEPENDENT_CODE=ON .
sudo make -j8 install 
cd
rm -rf tinyxml_2_6_2.zip tinyxml

# Build and install google logging libraries
cd
wget https://github.com/google/glog/archive/v0.3.5.tar.gz
tar -tzvf v0.3.5.tar.gz 
cd glog-0.3.5/
CFLAGS="-O2 -fPIC" CXXFLAGS="-O2 -fPIC" LDFLAGS="-fPIC" ./configure --host=arm-frc-linux-gnueabi --prefix=/usr/arm-frc-linux-gnueabi/usr/local 
sudo make -j8 install
rm -rf  v0.3.5.tar.gz glog-0.3.5

# Build and install qhull libraries
cd
wget http://www.qhull.org/download/qhull-2015-src-7.2.0.tgz
tar -xzvf qhull-2015-src-7.2.0.tgz
cd qhull-2015.2/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`/usr/bin/frc-cmake-toolchain` -DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi .
sudo make -j8 install
cd
rm -rf qhull-2015-src-7.2.0.tgz qhull-2015.2

# Build and install assimp libraries
cd
git clone https://github.com/assimp/assimp.git 
cd assimp
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`/usr/bin/frc-cmake-toolchain`-DCMAKE_INSTALL_PREFIX:PATH=/usr/arm-frc-linux-gnueabi  .
sudo make -j8 install
cd
rm -rf assimp

# Build and install UUID libraries
cd
wget https://downloads.sourceforge.net/project/libuuid/libuuid-1.0.3.tar.gz
tar -xzvf libuuid-1.0.3.tar.gz
cd libuuid-1.0.3
./configure --host=arm-frc-linux-gnueabi --prefix=/usr/arm-frc-linux-gnueabi/usr/local 
sudo make install
cd
rm -rf libuuid-1.0.3.tar.gz libuuid-1.0.3


