#include "path_to_goal/path_to_goal.h"

class PathAction
{
protected:
	actionlib::SimpleActionServer<path_to_goal::PathAction> as_;
	std::string action_name_;

	path_to_goal::PathFeedback feedback_;
	path_to_goal::PathResult result_;

public: 
	PathAction(std::string name, ros::NodeHandle n_) : 
		as_(n_, name, boost::bind(&PathAction::executeCB, this, _1), false),
		action_name_(name)
	{
		as_.start();
	}

	~PathAction(void)
	{
	}

	void executeCB(const path_to_goal::PathGoalConstPtr &goal) //make a state thing so that it just progresses to the next service call
	{
		bool success = true;

		base_trajectory::GenerateSpline srvBaseTrajectory;
		srvBaseTrajectory.request.points.resize(1);

		swerve_point_generator::FullGenCoefs traj;

		ros::Duration time_to_run = ros::Duration(5); //TODO: make this an actual thing

		switch(goal->goal_index) {
		case 0 : //user input data
		{
			//x-movement
			srvBaseTrajectory.request.points[0].positions.push_back(goal->x);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//y-movement
			srvBaseTrajectory.request.points[0].positions.push_back(goal->y);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//z-rotation
			srvBaseTrajectory.request.points[0].positions.push_back(goal->rotation);
			srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
			srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
			//time for profile to run
			srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
		}
		case 1 : //cube data 
		{
			if (cube_location.location.size() == 0)
			{
				ROS_ERROR_STREAM("NO CUBES FOUND - generateCoefs");
				success = false;
				break;
			}

			ROS_INFO_STREAM("x = " << cube_location.location[0].x);
			ROS_INFO_STREAM("z = " << cube_location.location[0].z);
			
			//x-movement
			srvBaseTrajectory.request.points[0].positions.push_back(cube_location.location[0].x); //TODO: are these in the right places? 
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//y-movement
			srvBaseTrajectory.request.points[0].positions.push_back(cube_location.location[0].z);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//z-rotation
			srvBaseTrajectory.request.points[0].positions.push_back(0);
			srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
			srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
			//time for profile to run
			srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
		}
		case 2 : //exchange data
		{
			if (qr_location.location.size() == 0)
			{
				ROS_ERROR_STREAM("NO CUBES FOUND - generateCoefs");
				success = false;
				break;
			}

			ROS_INFO_STREAM("x = " << qr_location.location[0].x);
			ROS_INFO_STREAM("z = " << qr_location.location[0].z);
			
			//x-movement
			srvBaseTrajectory.request.points[0].positions.push_back(qr_location.location[0].x); //TODO: are these in the right places? 
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//y-movement
			srvBaseTrajectory.request.points[0].positions.push_back(qr_location.location[0].z);
			srvBaseTrajectory.request.points[0].velocities.push_back(0);
			srvBaseTrajectory.request.points[0].accelerations.push_back(0);
			//z-rotation
			srvBaseTrajectory.request.points[0].positions.push_back(0);
			srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
			srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
			//time for profile to run
			srvBaseTrajectory.request.points[0].time_from_start = time_to_run;
		}
		default :
			ROS_ERROR_STREAM("goal index of " << goal->goal_index << " is not recognized");
		}

		
		static bool running = false;

		if (!running || outOfPoints)
		{
			running = true;
			if(!spline_gen.call(srvBaseTrajectory))
				ROS_INFO_STREAM("spline_gen died");
			else if (!generateTrajectory(srvBaseTrajectory, traj))
				ROS_INFO_STREAM("generateTrajectory died");
			else if (!runTrajectory(traj.response))
				ROS_INFO_STREAM("runTrajectory died");
		}
		else
			ROS_INFO_STREAM("path_to_cube is already running");
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_server");
	ros::NodeHandle n;
	PathAction path("path_action", n);

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = 1;
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_controller = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);
	cube_sub = n.subscribe("/cube_detect/cube_detect_msg", 10, &cubeCallback);
	talon_sub = n.subscribe("/frcrobot/talon_states", 10, talonStateCallback);

	ros::spin();

	return 0;
}

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

void QRCallback(cube_detection::CubeDetection sub_location)
{
	qr_location.location.resize(sub_location.location.size());

	if (sub_location.location.size() > 0)
	{
		qr_location.location[0].x = sub_location.location[0].x;
		qr_location.location[0].y = sub_location.location[0].y;
		qr_location.location[0].z = sub_location.location[0].z;
		qr_location.angle = sub_location.angle;
	}
	else
	{
		ROS_ERROR_STREAM("NO EXCHANGE FOUND");
	}
}
