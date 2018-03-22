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
#include <swerve_point_generator/Coefs.h>
#include <swerve_point_generator/FullGen.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <swerve_point_generator/GenerateTrajectory.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <XmlRpcValue.h>
#include <vector>
enum Action {deploy_intake, undeploy_intake, intake_cube, exchange_cube, default_config, intake_config, exchange_config, switch_config, low_scale_config, mid_scale_config, high_scale_config, over_back_config, release_clamp};

typedef struct action
{
    double time;
    std::vector<Action> actions;
} action_struct;

typedef struct cmd_vel
{
    double duration;
    double x;
    double y;
} cmd_vel_struct;

struct full_mode
{
	swerve_point_generator::FullGenCoefs srv_msg;
	std::vector<action_struct> actions;
	bool exists;

	full_mode(void): exists(false) {}
};

struct cmd_vel_mode
{
    std::vector<cmd_vel_struct> segments;
    bool exists;
};
typedef std::vector<std::vector<std::vector<full_mode>>> mode_list;
typedef std::vector<std::vector<std::vector<cmd_vel_mode>>> cmd_vel_list;


typedef struct mode
{
    mode_list profiled_modes;
    cmd_vel_list cmd_vel_modes;
} Modes;

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
bool intakeOut(void);
bool parkingConfig(void);
bool generateTrajectory(full_mode &trajectory);
bool bufferTrajectory(const swerve_point_generator::FullGenCoefs::Response &traj);
bool runTrajectory(void);
void auto_mode_cb(const ros_control_boilerplate::AutoMode::ConstPtr &AutoMode);
void match_data_cb(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData);
void run_auto(int auto_select, int auto_mode, int layout, int start_pos, double initial_delay, const std::vector<Action> &times);


mode_list load_all_trajectories(int max_mode_num, int max_start_pos_num, ros::NodeHandle &auto_data);
