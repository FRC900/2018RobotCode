#include <ros/ros.h>
#include <cube_detection/CubeDetection.h>
#include <swerve_point_generator/FullGenCoefs.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <base_trajectory/GenerateSpline.h>
#include <talon_state_controller/TalonState.h>
#include <robot_visualizer/ProfileFollower.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

//namespace path_to_cube
//{
/*
 * to test: roslaunch 2018_main_frcrobot
 * play only the /zed_goal/left/image_rect_color and /zed_goal/depth/depth_registered topics
 * rosrun cube_detection cube_detection node
 * rosrun path_to_cube path_to_cube node
 * rosservice call /path_to_cube/cmd_service
 */
ros::Subscriber cube_sub;
ros::ServiceServer cmd_service;
ros::ServiceClient point_gen;
ros::ServiceClient swerve_controller;
ros::ServiceClient spline_gen;
ros::ServiceClient VisualizeService;
cube_detection::CubeDetection cube_location;
bool outOfPoints;
ros::Subscriber talon_sub;
bool command;

void cubeCallback(cube_detection::CubeDetection sub_location)
{
	cube_location.location.resize(sub_location.location.size());

	//for(int i; i < cube_location.location.size(); i++)
	//{
		cube_location.location[0].x = sub_location.location[0].x;
		cube_location.location[0].y = sub_location.location[0].y;
		cube_location.location[0].z = sub_location.location[0].z;
	//}
	cube_location.angle = sub_location.angle;
}

void talonStateCallback(const talon_state_controller::TalonState &talon_state)
{
	static size_t bl_drive_idx = std::numeric_limits<size_t>::max();

	if (bl_drive_idx >= talon_state.name.size())
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if(talon_state.name[i] == "bl_drive")
			{
				bl_drive_idx = i;
				break;
			}
		}
	}

	if (bl_drive_idx < talon_state.custom_profile_status.size())
		outOfPoints = talon_state.custom_profile_status[bl_drive_idx].outOfPoints;
}

float coerce(float x)
{
	return ((x>0.5) ? x : .5);
}

bool generateCoefs(base_trajectory::GenerateSpline &srvBaseTrajectory)
{
	ros::Duration time_to_run = ros::Duration(5);
	srvBaseTrajectory.request.points.resize(1);

	//x-movement
	srvBaseTrajectory.request.points[0].positions.push_back(cube_location.location[0].x);
	srvBaseTrajectory.request.points[0].velocities.push_back(0);
	srvBaseTrajectory.request.points[0].accelerations.push_back(0);
	//y-movement
	float coerced = coerce(cube_location.location[0].z);
	srvBaseTrajectory.request.points[0].positions.push_back(coerced);
	srvBaseTrajectory.request.points[0].velocities.push_back(0);
	srvBaseTrajectory.request.points[0].accelerations.push_back(0);
	//z-rotation
	srvBaseTrajectory.request.points[0].positions.push_back(0);
	srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
	srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
	//time for profile to run
	srvBaseTrajectory.request.points[0].time_from_start = time_to_run;

	if(!(spline_gen.call(srvBaseTrajectory)))
		return false;
	else
		return true;
}

bool generateTrajectory(const base_trajectory::GenerateSpline &srvBaseTrajectory, swerve_point_generator::FullGenCoefs &traj)
{
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

bool cmdSubService(std_srvs::Empty::Request &/*command*/, std_srvs::Empty::Response &/*res*/)
{
	ROS_INFO_STREAM("callback is running");
	static bool running = false;
	base_trajectory::GenerateSpline srvBaseTrajectory;
	swerve_point_generator::FullGenCoefs traj;

	if (!running || outOfPoints)
	{
		if(!generateCoefs(srvBaseTrajectory))
			ROS_INFO_STREAM("generateCoefs died");
		else if (!generateTrajectory(srvBaseTrajectory, traj))
			ROS_INFO_STREAM("generateTrajectory died");
		else if (!runTrajectory(traj.response))
			ROS_INFO_STREAM("runTrajectory died");
		running = true;
	}
	else
		ROS_INFO_STREAM("path_to_cube is already running");
	ROS_INFO_STREAM("callback SUCCESS");
}

int main(int argc, char **argv)
{
	ROS_INFO_STREAM("I've been inited");

	ros::init(argc, argv, "path_to_cube");
	ros::NodeHandle n;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";

	cube_sub = n.subscribe("/cube_detect/cube_detect_msg", 10, &cubeCallback);
	talon_sub = n.subscribe("/frcrobot/talon_states", 10, talonStateCallback);
	cmd_service = n.advertiseService("cmd_service", cmdSubService);

	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_controller = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);

	//ros::Timer timer = n.createTimer(ros::Duration(.1), callback);

	ros::spin();
	
	return 0;
}
//} //namespace
