# Setup ROS for Local Development

source /opt/ros/kinetic/setup.bash
source ~/2017VisionCode/zebROS_ws/install_isolated/setup.bash
export ROS_MASTER_URI=http://10.9.0.8:5802

# Set to local device IP?
export ROS_IP=10.9.0.22
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

