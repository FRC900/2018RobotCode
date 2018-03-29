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
#include <talon_swerve_drive_controller/SwervePointSet.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <XmlRpcValue.h>
#include <vector>
enum Action {deploy_intake, undeploy_intake, intake_cube, exchange_cube, default_config, intake_config, exchange_config, switch_config, low_scale_config, mid_scale_config, high_scale_config, over_back_config, custom_config, release_clamp};

struct ActionSetpoint
{
	double x;
	double y;
	bool up_or_down;

};

struct ActionStruct
{
    double time;
    Action action;
	ActionSetpoint action_setpoint;
};

struct CmdVel
{
    double duration;
    double x;
    double y;
};

struct FullMode
{
	std::vector<swerve_point_generator::FullGenCoefs> srv_msgs;
	int num_srv_msgs;
	std::vector<ActionStruct> actions;
	std::vector<int> wait_ids;
	bool exists;

	FullMode(void): exists(false), num_srv_msgs(0) {}
};

struct CmdVelMode
{
    std::vector<CmdVel> segments;
    bool exists;

	CmdVelMode(void): exists(false) {}
};

typedef std::vector<std::vector<std::vector<FullMode>>> ModeList;
typedef std::vector<std::vector<std::vector<CmdVelMode>>> CmdVelList; //Also should maybe have another dimensition and wait for completions?

struct Modes
{
    ModeList profiled_modes;
    CmdVelList cmd_vel_modes;
};

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
bool runTrajectory(void);
void auto_mode_cb(const ros_control_boilerplate::AutoMode::ConstPtr &AutoMode);
void match_data_cb(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData);
void run_auto(int auto_select, int layout, int start_pos, double initial_delay, const FullMode &auto_run_data, std::vector<int> start_of_buffer_ids);
void run_auto(int auto_select, int layout, int start_pos, double initial_delay, const std::vector<CmdVel> &segments);


ModeList load_all_trajectories(int max_mode_num, int max_start_pos_num, ros::NodeHandle &auto_data);
