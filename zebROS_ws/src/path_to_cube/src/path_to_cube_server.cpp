#include "path_to_cube/path_to_cube.h"

class PathAction
{
protected:
	ros::NodeHandle n_;
	actionlib::SimpleActionServer<path_to_cube::PathAction> as_;
	std::string action_name_;

	path_to_cube::PathFeedback feedback_;
	path_to_cube::PathResult result_;

public: 
	PathAction(std::string name) : 
		as_(nh_, name, boost::bind(&PathAction::executeCB, this, _1), false),
		action_name_(name)
	{
		as_.start();
	}

	~PathAction(void)
	{
	}

	void executeCB(const path_to_cube::PathGoalConstPtr &goal) //make a state thing so that it just progresses to the next service call
	{
		bool success = true;
		
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
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_server");

	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_controller = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);

	PathAction path("path_action");
	ros::spin();

	return 0;
}

