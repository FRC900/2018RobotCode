#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "elevator_controller/CubeState.h"
#include <std_srvs/Empty.h>
#include "auto_preseason/Angle.h"
#include "cube_detection/CubeDetection.h"
#include "robot_visualizer/ProfileFollower.h"
#include "talon_swerve_drive_controller/MotionProfilePoints.h"

static bool cube_state;
static cube_detection::CubeDetection cube_detection_msg;

void cube_state_callback(const elevator_controller::CubeState &msg);
void cube_detection_callback(const cube_detection::CubeDetection &msg);
