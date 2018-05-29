#!/bin/bash

set -e
set -o pipefail

# IP addresses of the roboRIO and Jetson to deploy code on.
ROBORIO_ADDR=10.9.0.2
JETSON_ADDR=10.9.0.8

# Environment to deploy to (prod or dev).
INSTALL_ENV=dev

# Whether we're doing a build or just updating symlinks.
UPDATE_LINKS_ONLY=0

# Location of the code.
LOCAL_CLONE_LOCATION=$HOME/2018RobotCode
ROS_CODE_LOCATION=$LOCAL_CLONE_LOCATION/zebROS_ws
RSYNC_OPTIONS=""

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
        exit 1
        ;;
    -p|--prod)
        INSTALL_ENV=prod
        shift
        ;;
    -d|--dev)
        INSTALL_ENV=dev
        shift
        ;;
    -o|--one-dir-sync)
        RSYNC_OPTIONS="--delete"
        shift
        ;;
    -u|--update-links-only)
        UPDATE_LINKS_ONLY=1
        shift
	;;
    *) # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

# Directory paths on the Jetson and roboRIO.
RIO_CLONE_LOCATION=/home/admin/2018RobotCode
RIO_ENV_LOCATION=$RIO_CLONE_LOCATION.$INSTALL_ENV
RIO_ROS_CODE_LOCATION=$RIO_ENV_LOCATION/zebROS_ws
RIO_INSTALL_LOCATION=$RIO_ROS_CODE_LOCATION/install_isolated

JETSON_CLONE_LOCATION=/home/ubuntu/2018RobotCode
JETSON_ENV_LOCATION=$JETSON_CLONE_LOCATION.$INSTALL_ENV
JETSON_ROS_CODE_LOCATION=$JETSON_ENV_LOCATION/zebROS_ws

update_links() {
    # Update symlinks on the roboRIO and Jetson.
    ssh $ROBORIO_ADDR "rm $RIO_CLONE_LOCATION && \
        ln -s $RIO_ENV_LOCATION $RIO_CLONE_LOCATION"
    ssh $JETSON_ADDR "rm $JETSON_CLONE_LOCATION && \
        ln -s $JETSON_ENV_LOCATION $JETSON_CLONE_LOCATION"
    echo "Symlinks updated."
}

check_clockdiff() {
    #read -r -a TIMEDIFF <<< `clockdiff $1`
    #if [[ ${#TIMEDIFF[@]} -ge 3 ]]; then
    #    echo "${TIMEDIFF[1]} msec diff"
    #else
    #    echo "Could not parse clockdiff output!"
    #    exit 1
    #fi
    LOCALDATE=`date +%s`
    REMOTEDATE=`ssh $1 date +%s`
    let TIMEDIFF=$LOCALDATE-$REMOTEDATE
    TIMEDIFF=${TIMEDIFF#-}
    if [[ $TIMEDIFF -ge 600 ]]; then
        REMOTE_TIME=`ssh $1 date`
        echo "Clock difference greater than 10 minutes."
        echo "    Local time: `date`"
        echo "    Time on $2: $REMOTE_TIME"
        exit 1
    fi
}

update_links
if [ $UPDATE_LINKS_ONLY -ne 0 ]; then
    exit 0
fi

echo "Checking time synchronization..."
check_clockdiff "$ROBORIO_ADDR" "roboRIO"
check_clockdiff "$JETSON_ADDR" "Jetson"
echo "Time synchronized."

# Bidirectional synchronization of the selected environment.
echo "Synchronizing local changes TO $INSTALL_ENV environment."
scp $ROS_CODE_LOCATION/ROSJetsonMaster.sh $JETSON_ADDR:$JETSON_ROS_CODE_LOCATION
scp $ROS_CODE_LOCATION/ROSJetsonMaster.sh $ROBORIO_ADDR:$RIO_ROS_CODE_LOCATION
rsync -avzru $RSYNC_OPTIONS --exclude '.git' --exclude 'zebROS_ws/build*' \
    --exclude 'zebROS_ws/devel*' --exclude 'zebROS_ws/install*' \
    --exclude '*~' --exclude '*.sw[op]' --exclude '*CMakeFiles*' \
    $LOCAL_CLONE_LOCATION/ $JETSON_ADDR:$JETSON_ENV_LOCATION/
if [ $? -ne 0 ]; then
    echo "Failed to synchronize source code TO $INSTALL_ENV on Jetson!"
    exit 1
fi
echo "Synchronization complete"
if [ ${#RSYNC_OPTIONS} -eq 0 ] ; then
    echo "Synchronizing remote changes FROM $INSTALL_ENV environment."
    rsync -avzru --exclude '.git' --exclude 'zebROS_ws/build*' \
        --exclude 'zebROS_ws/devel*' --exclude 'zebROS_ws/install*' \
        --exclude '*~' --exclude '*.sw[op]'  --exclude '*CMakeFiles*' \
        $JETSON_ADDR:$JETSON_ENV_LOCATION/ $LOCAL_CLONE_LOCATION/
    if [ $? -ne 0 ]; then
        echo "Failed to synchronize source code FROM $INSTALL_ENV on Jetson!"
        exit 1
    fi
fi
echo "Synchronization complete"

# Run local roboRIO cross build as one process.
# Then synchronize cross build products to roboRIO.
(echo "Starting roboRIO cross build" &&
bash -c "cd $ROS_CODE_LOCATION && \
    source /usr/arm-frc-linux-gnueabi/opt/ros/kinetic/setup.bash && \
    ./cross_build.sh" && \
echo "roboRIO cross build complete" && \
echo "Synchronizing $INSTALL_ENV cross build to roboRIO" && \
ssh $ROBORIO_ADDR "/etc/init.d/nilvrt stop" && \
rsync -avz --delete $ROS_CODE_LOCATION/install_isolated/ \
    $ROBORIO_ADDR:$RIO_INSTALL_LOCATION && /
echo "Synchronization complete") &

# Run Jetson native build as a separate process.
(echo "Starting Jetson native build" && \
ssh $JETSON_ADDR "cd $JETSON_CLONE_LOCATION/zebROS_ws && \
    source /opt/ros/kinetic/setup.bash && \
    catkin_make" && \
echo "Jetson native build complete") &

wait

update_links
echo "FINISHED SUCCESSFULLY"

