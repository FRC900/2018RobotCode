FROM ubuntu:16.04

MAINTAINER Team 900

# Needed for resolvconf /etc/resolv.conf Device or resource busy errors?
#echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections && \

# docker run -it --privileged --net=host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ubuntu:16.04 /bin/bash

RUN apt update && \
    apt upgrade -y && \
    apt install -y ubuntu-minimal ubuntu-standard software-properties-common ; \
    apt purge -y plymouth plymouth-theme-ubuntu-text libdrm2 libplymouth4 && \
    apt-add-repository -y ppa:wpilib/toolchain && \
    apt-add-repository -y ppa:wpilib/toolchain-beta && \
    apt-add-repository -y ppa:webupd8team/java && \
    apt update && \
    apt install -y git libc6-i386 curl jstest-gtk gradle oracle-java8-installer frc-toolchain meshlab cmake libprotobuf-dev libprotoc-dev protobuf-compiler ninja-build sip-dev python-empy libtinyxml2-dev libswt-gtk-3-jni libswt-gtk-3-java && \
    useradd -ms /bin/bash -G sudo ubuntu

# passwd ubuntu

USER ubuntu

RUN cd && \
    wget http://www.ctr-electronics.com//downloads/lib/CTRE_FRCLibs_NON-WINDOWS_v4.4.1.14.zip && \
    mkdir ctre && \
    cd ctre && \
    unzip ../CTRE_FRCLibs_NON-WINDOWS_v4.4.1.14.zip && \
    cp -r cpp ~/wpilib/user && \
    cd .. && \
    rm -rf ctre CTRE_FRCLibs_NON-WINDOWS_v4.4.1.14.zip && \




#    mkdir -p ~/Downloads && \
#    cd ~/Downloads && \
#    wget http://mirror.csclub.uwaterloo.ca/eclipse/oomph/epp/neon/R/eclipse-inst-linux64.tar.gz && \
#    tar zxvf eclipse-inst-linux64.tar.gz && \
#    cd eclipse-installer && \
#    ./eclipse-inst

