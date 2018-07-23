#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "elevator_controller/CubeState.h"
#include <std_srvs/Empty.h>
#include "auto_preseason/Angle.h"
#include "cube_detection/CubeDetection.h"
#include "robot_visualizer/ProfileFollower.h"
#include "talon_swerve_drive_controller/MotionProfilePoints.h"
#include "elevator_controller/Intake.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "path_to_cube/PathAction.h"
#include "behaviors/IntakeAction.h"
#include "zbar_ros/PathAction.h"

static bool cube_found;
static bool haz_cube;
static cube_detection::CubeDetection cube_detection_msg;
static bool qr_found;
static double cube_dist;
static double exchange_dist;
static cube_detection::CubeDetection exchange_detection_msg;

static double min_pathing_dist = 0.3;

void cube_state_callback(const elevator_controller::CubeState &msg);
void cube_detection_callback(const cube_detection::CubeDetection &msg);
void exchange_detection_callback(const cube_detection::CubeDetection &msg);
