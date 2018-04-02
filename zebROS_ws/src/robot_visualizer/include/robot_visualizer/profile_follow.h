#include <ros/ros.h>
#include <ros/console.h>
#include <robot_visualizer/ProfileFollower.h>
#include <robot_visualizer/RobotVisualizeState.h>
#include <talon_state_controller/TalonState.h>
//#include <trajectory_msgs/JointTrajh>

#include <cmath>

bool follow_service(robot_visualizer::ProfileFollower::Request &req, robot_visualizer::ProfileFollower::Response &res);

void talon_cb(const talon_state_controller::TalonState &msg);
