# Setup ROS for Turtlebot Development

source /opt/ros/kinetic/setup.bash
source ~/2017RioPreseason/devel/setup.bash
export ROS_MASTER_URI=http://10.0.0.120:11311

# Set to local device IP?
export ROS_IP=$(hostname -I)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
