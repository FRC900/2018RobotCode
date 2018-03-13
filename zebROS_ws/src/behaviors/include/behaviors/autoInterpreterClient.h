#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/AutoMode.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/RobotAction.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"

#include "std_msgs/Bool.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/ElevatorControlS.h"
#include "std_srvs/Empty.h"
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <talon_swerve_drive_controller/GenerateTrajectory.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <talon_swerve_drive_controller/FullGen.h>
#include <talon_swerve_drive_controller/FullGenCoefs.h>
#include <talon_swerve_drive_controller/Coefs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <XmlRpcValue.h>
#include <vector>


bool defaultConfig(void);
bool intakeConfig(void);
bool switchConfig(void);
bool highScale(void);
bool midScale(void);
bool lowScale(void);
bool stopIntake(void);
bool releaseClamp(void);
bool clamp(void);
bool releaseIntake(void);
bool parkingConfig(void);
bool generateTrajectory(int auto_mode,int layout,int start_pos);
void runTrajectory(int auto_mode);
void auto_mode_cb(const ros_control_boilerplate::AutoMode::ConstPtr &AutoMode);
void match_data_cb(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData);
void run_auto(int auto_mode);
void bufferTrajectory(int auto_mode, int layout, int start_pos);
void load_all_trajectories(int max_mode_num, int max_start_pos_num, ros::NodeHandle &auto_data);
