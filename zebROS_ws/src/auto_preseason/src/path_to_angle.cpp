#include <auto_preseason/path_to_angle.h>
#include <base_trajectory/GenerateSpline.h>
#include <swerve_point_generator/FullGenCoefs.h>

bool outOfPoints = true;
bool orient_running = false;
float desired_angle;
ros::ServiceClient VisualizeService;
ros::ServiceClient point_gen;
ros::ServiceClient swerve_control;
ros::ServiceClient spline_gen;
ros::ServiceServer turn_to_angle;
ros::ServiceServer snap_to_angle;
float navX_angle;

bool generateCoefs(const double angle_diff, const ros::Duration &time_to_run, base_trajectory::GenerateSpline &srvBaseTrajectory)
{
	if (angle_diff == 0)
		return false;

	if (time_to_run.toSec() <= 0)
	{
		ROS_ERROR("generateCoefs() called with <= 0 time_to_run");
		return false;
	}
	srvBaseTrajectory.request.points.resize(1); //only need one endpoint -- final orientation of robot
	
	//x-movement (not moving at all)
	srvBaseTrajectory.request.points[0].positions.push_back(0.05);
	srvBaseTrajectory.request.points[0].velocities.push_back(0);
	srvBaseTrajectory.request.points[0].accelerations.push_back(0);
	//y-movement (not moving at all)
	srvBaseTrajectory.request.points[0].positions.push_back(0.05);
	srvBaseTrajectory.request.points[0].velocities.push_back(0);
	srvBaseTrajectory.request.points[0].accelerations.push_back(0);
	//z-rotation
	srvBaseTrajectory.request.points[0].positions.push_back(angle_diff);
	srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
	srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
	//time for profile to run
	srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
	
	if(!spline_gen.call(srvBaseTrajectory))
		return false;
	else
		return true;
}

bool generateTrajectory(const base_trajectory::GenerateSpline &srvBaseTrajectory, swerve_point_generator::FullGenCoefs &traj) 
{
	traj.request.orient_coefs.resize(1);
	traj.request.x_coefs.resize(1);
	traj.request.y_coefs.resize(1);
	std::vector<double> temp_o = {-18.843733642, 47.1064309261, -31.3984809261, -0.00580635796197, 0, 0};
	std::vector<double> temp_x = {0, 0, 0, 0, 0.01, 0};
	std::vector<double> temp_y = {0, 0, 0, 0, 0, 0};
	for(size_t i = 0; i < temp_o.size(); i++)
	{
		traj.request.orient_coefs[0].spline.push_back(temp_o[i]);
		traj.request.x_coefs[0].spline.push_back(temp_x[i]);
		traj.request.y_coefs[0].spline.push_back(temp_y[i]);
	}
	//traj.request.x_coefs[0] = {0, 0, 0, 0, 0, 0};//srvBaseTrajectory.response.x_coefs[1];
	//traj.request.y_coefs[0] = {0, 0, .05, 0, 0, 0};//srvBaseTrajectory.response.y_coefs[1];
	traj.request.spline_groups.push_back(1);
	traj.request.wait_before_group.push_back(.16);
	traj.request.t_shift.push_back(0);
	traj.request.flip.push_back(false);
	traj.request.end_points.push_back(1);
	//traj.request.end_points = srvBaseTrajectory.response.end_points;
	traj.request.initial_v = 0;
	traj.request.final_v = 0;
	traj.request.x_invert.push_back(0);

	if (traj.request.end_points.empty())
		ROS_WARN("request end points are empty");
	//else
		//ROS_WARN("request end points are not empty????????");

	if (srvBaseTrajectory.response.end_points.empty())
		ROS_INFO_STREAM("things are broken");

	ROS_ERROR("Here!");
	if(!point_gen.call(traj))
		return false;
	else
		return true;
}

bool runTrajectory(const swerve_point_generator::FullGenCoefs::Response &traj)
{

	ROS_INFO_STREAM("traj: " << traj.dt);

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
    
    if (!swerve_control.call(swerve_control_srv))
		return false;
	else
		return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turn_to_angle_services");
	ros::NodeHandle n;

	ROS_INFO_STREAM("outOfPoints = " << outOfPoints);
	static bool orient_running = false;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);    
	turn_to_angle = n.advertiseService("turn_to_angle", turn_to_angle_srv);
	snap_to_angle = n.advertiseService("snap_to_angle", snap_to_angle_srv);

	const double max_rotational_velocity = 8.8; //radians/sec TODO: find this in config
	const ros::Duration time_to_run((fabs(desired_angle) / max_rotational_velocity) * .5); //TODO: needs testing

	base_trajectory::GenerateSpline srvBaseTrajectory;
	swerve_point_generator::FullGenCoefs traj;
	
	if (!generateCoefs(desired_angle, time_to_run, srvBaseTrajectory)) //generate coefficients for the spline from the endpoints 
		ROS_INFO_STREAM("spline_gen died in teleopJoystickCommands generateCoefs");
	else if (!generateTrajectory(srvBaseTrajectory, traj)) //generate a motion profile from the coefs
		ROS_INFO_STREAM("point_gen died in teleopJoystickCommands generateTrajectory");
	else if (!runTrajectory(traj.response)) //run on swerve_control
		ROS_ERROR("swerve_control failed in teleopJoystickCommands runTrajectory");
	else
		orient_running = true;
}

//make a service to snap to angle
bool snap_to_angle_srv(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/) {
	if(!orient_running || outOfPoints)
	{
		orient_running = false;
		const double angle = -navX_angle - M_PI / 2;
		//const double angle = M_PI; //for testing
		ROS_INFO_STREAM("angle = " << angle);
		// TODO: look at using ros::angles package
		//const double least_dist_angle = round(angle/(M_PI/2))*M_PI/2;
		const double least_dist_angle = angle + 2* M_PI;

		desired_angle = least_dist_angle - angle;
	}
	else 
	{
		ROS_INFO_STREAM("Can't run orient, it's already running");
		if (outOfPoints)
			orient_running = false;
	}
}

//make a service to snap to closest angle
bool turn_to_angle_srv(auto_preseason::Angle::Request &req, const auto_preseason::Angle::Response &/*res*/)
{
	if(!orient_running || outOfPoints)
	{
		orient_running = false;
		desired_angle = req.angle;
	}
	else
	{
		ROS_INFO_STREAM("Can't run orient, it's already running");
		if(outOfPoints)
			orient_running = false;
	}
}


void talonStateCallback(const talon_state_controller::TalonState &talon_state)
{
	// TODO : This shouldn't be hard-coded
	static size_t bl_angle_idx = std::numeric_limits<size_t>::max();

	if (bl_angle_idx >= talon_state.name.size())
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if (talon_state.name[i] == "bl_angle")
			{
				bl_angle_idx = i;
				break;
			}
		}
	}

	if (bl_angle_idx < talon_state.custom_profile_status.size())
		outOfPoints = talon_state.custom_profile_status[bl_angle_idx].outOfPoints;
}
