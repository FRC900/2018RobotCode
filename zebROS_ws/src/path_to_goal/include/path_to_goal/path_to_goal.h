#include <ros/ros.h>
#include <cube_detection/CubeDetection.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <base_trajectory/GenerateSpline.h>
#include <talon_state_controller/TalonState.h>
#include <robot_visualizer/ProfileFollower.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <path_to_goal/PathAction.h>
#include <actionlib/server/simple_action_server.h>

ros::Subscriber cube_sub;
ros::ServiceServer cmd_service;
ros::ServiceClient point_gen;
ros::ServiceClient swerve_controller;
ros::ServiceClient spline_gen;
ros::ServiceClient VisualizeService;
cube_detection::CubeDetection cube_location;
cube_detection::CubeDetection qr_location;
bool outOfPoints;
ros::Subscriber talon_sub;
bool command;

float coerce(float x)
{
	return ((x>0.05) ? x : .05);
}

bool generateTrajectory(const base_trajectory::GenerateSpline &srvBaseTrajectory, swerve_point_generator::FullGenCoefs &traj)
{
	ROS_INFO_STREAM("started generateTrajectory");
	traj.request.orient_coefs.resize(1);
	traj.request.x_coefs.resize(1);
	traj.request.y_coefs.resize(1);

	for(size_t i = 0; i < srvBaseTrajectory.response.orient_coefs[0].spline.size(); i++)
	{
		traj.request.orient_coefs[0].spline.push_back(srvBaseTrajectory.response.orient_coefs[1].spline[i]);
		traj.request.x_coefs[0].spline.push_back(srvBaseTrajectory.response.x_coefs[1].spline[i]);
		traj.request.y_coefs[0].spline.push_back(srvBaseTrajectory.response.y_coefs[1].spline[i]);
	}

	traj.request.spline_groups.push_back(1);
	traj.request.wait_before_group.push_back(.16);
	traj.request.t_shift.push_back(0);
	traj.request.flip.push_back(false);
	traj.request.end_points.push_back(1);
	traj.request.end_points.resize(1);
	traj.request.end_points[0] = srvBaseTrajectory.response.end_points[1];
	traj.request.initial_v = 0;
	traj.request.final_v = 0;
	traj.request.x_invert.push_back(0);

	if(!point_gen.call(traj))
		return false;
	else
		return true;
}

bool runTrajectory(const swerve_point_generator::FullGenCoefs::Response &traj)
{
	ROS_INFO_STREAM("started runTrajectory");
	//visualization stuff
	robot_visualizer::ProfileFollower srv_viz_msg;
	srv_viz_msg.request.joint_trajectories.push_back(traj.joint_trajectory);

	srv_viz_msg.request.start_id = 0;

	if(!VisualizeService.call(srv_viz_msg))
	{
		ROS_ERROR("failed to call viz srv");
	}
	else
	{
		ROS_ERROR("succeded in call to viz srv");
	}

	talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
	swerve_control_srv.request.profiles.resize(1);
    swerve_control_srv.request.profiles[0].points = traj.points;
    swerve_control_srv.request.profiles[0].dt = 0.02;
    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.run = true;
    swerve_control_srv.request.profiles[0].slot = 0;
    
    if (!swerve_controller.call(swerve_control_srv))
		return false;
	else
		return true;
}

void cubeCallback(cube_detection::CubeDetection sub_location);

void talonStateCallback(const talon_state_controller::TalonState &talon_state);

void QRCallback(cube_detection::CubeDetection sub_location);
