#!/bin/bash -x

set -e
set -o pipefail

# IP addresses of the roboRIO and Jetson to deploy code on.
ROBORIO_ADDR=10.9.0.2
JETSON_ADDR=10.9.0.8

# Environment to deploy to (prod or dev).
INSTALL_ENV=dev

# Location of the code.
LOCAL_CLONE_LOCATION=$HOME/2018RobotCode
ROS_CODE_LOCATION=$LOCAL_CLONE_LOCATION/zebROS_ws
JETSON_CLONE_LOCATION=/home/ubuntu/2018RobotCode

usage() {
    echo "Usage: $0 [-d|-p]"
    exit 1
}

# Command line argument parsing.
POSITIONAL=()
while [[ $# -gt 0 ]] ; do
    key="$1"

    case $key in
    -h|--help)
        usage
        exit 0
        ;;
    -p|--prod)
        INSTALL_ENV=prod
        shift
        ;;
    -d|--dev)
        INSTALL_ENV=dev
        shift
        ;;
    *) # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

echo "Checking NTP synchronization."
ntp-wait -n 60 -s 1 -v
if [ $? -ne 0 ]; then
    echo "NTP not synchronized. Check NTP configuration and try again."
    exit 1
fi

# Bidirectional synchronization of the selected environment.
echo "Synchronizing local changes TO $INSTALL_ENV environment."
rsync -avzru --exclude '.git' --exclude 'zebROS_ws/build*' \
    --exclude 'zebROS_ws/devel*' --exclude 'zebROS_ws/install*' \
    $LOCAL_CLONE_LOCATION/ $JETSON_ADDR:$JETSON_CLONE_LOCATION.$INSTALL_ENV/
if [ $? -ne 0 ]; then
    echo "Failed to synchronize source code TO $INSTALL_ENV on Jetson!"
    exit 1
fi
echo "Synchronizing remote changes FROM $INSTALL_ENV environment."
rsync -avzru --exclude '.git' --exclude 'zebROS_ws/build*' \
    --exclude 'zebROS_ws/devel*' --exclude 'zebROS_ws/install*' \
    $JETSON_ADDR:$JETSON_CLONE_LOCATION.$INSTALL_ENV/ $LOCAL_CLONE_LOCATION/
if [ $? -ne 0 ]; then
    echo "Failed to synchronize source code FROM $INSTALL_ENV on Jetson!"
    exit 1
fi

# TODO: Update remote symlink
# 2018RobotCode should be a symlink that points to either the prod or dev
# clone that we push to. Make sure it is a symlink before proceeding.
#if [ ! -h $LOCAL_CLONE_LOCATION ]; then
#    echo "$LOCAL_CLONE_LOCATION is not a symlink."
#fi

# Run local roboRIO cross build.
bash -c "cd $ROS_CODE_LOCATION && \
    source /usr/arm-frc-linux-gnueabi/opt/ros/kinetic/setup.bash && \
    ./cross_build.sh"

# TODO: rsync build products to roboRIO

# Run Jetson native build.
ssh $JETSON_ADDR "cd $JETSON_CLONE_LOCATION.$INSTALL_ENV/zebROS_ws && \
    source /opt/ros/kinetic/setup.bash && \
    catkin_make"

# TODO: rsync build products to Jetson's 2018RobotCode directory.

