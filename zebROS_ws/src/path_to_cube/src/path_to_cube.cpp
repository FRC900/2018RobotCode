#include <path_to_cube/path_to_cube.h>

//namespace path_to_cube
//{
/*
 * to test: roslaunch 2018_main_frcrobot
 * play only the /zed_goal/left/image_rect_color and /zed_goal/depth/depth_registered topics
 * rosrun cube_detection cube_detection node
 * rosrun path_to_cube path_to_cube node
 * rosservice call /path_to_cube/cmd_service
 */
void cubeCallback(cube_detection::CubeDetection sub_location)
{
	cube_location.location.resize(sub_location.location.size());

	if (sub_location.location.size() > 0)
	{
		cube_location.location[0].x = sub_location.location[0].x;
		cube_location.location[0].y = sub_location.location[0].y;
		cube_location.location[0].z = sub_location.location[0].z;
		cube_location.angle = sub_location.angle;
	}
	else
	{
		ROS_ERROR_STREAM("NO CUBES FOUND");
	}

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

bool cmdSubService(std_srvs::Empty::Request &/*command*/, std_srvs::Empty::Response &/*res*/)
{
	ROS_INFO_STREAM("started cmdSubService");
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
	cmd_service = n.advertiseService("path_service", cmdSubService);

	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_controller = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);

	//ros::Timer timer = n.createTimer(ros::Duration(.1), callback);

	ros::spin();
	
	return 0;
}
//} //namespace
