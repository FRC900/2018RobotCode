#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <base_trajectory/GenerateSpline.h>

class RobotBase
{
	private:
#if 0
		// Action client for the joint trajectory action 
		// used to trigger the arm movement action
typedef actionlib::SimpleActionClient< ::JointTrajectoryAction > TrajClient;
		TrajClient* traj_client_;
#endif

	public:
		//! Initialize the action client and wait for action server to come up
		/*RobotBase()
		{
#if 0
			// tell the action client that we want to spin a thread by default
			traj_client_ = new TrajClient("/base_trajectory/follow_joint_trajectory", true);

			// wait for action server to come up
			while(!traj_client_->waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the joint_trajectory_action server");
			}
#endif
		}

		//! Clean up the action client
		~RobotBase()
		{
#if 0
			delete traj_client_;
#endif
		}
		*/
		//! Generates a simple trajectory with two waypoints, used as an example
		/*! Note that this trajectory contains two waypoints, joined together
		  as a single trajectory. Alternatively, each of these waypoints could
		  be in its own trajectory - a trajectory can have one or more waypoints
		  depending on the desired application.
		  */
		trajectory_msgs::JointTrajectory genTrajectory(void)
		{
			//our goal variable
			trajectory_msgs::JointTrajectory trajectory;

			// First, the joint names, which apply to all waypoints
			trajectory.joint_names.push_back("x_linear_joint");
			trajectory.joint_names.push_back("y_linear_joint");
			trajectory.joint_names.push_back("z_rotation_joint");

			const size_t num_joints = trajectory.joint_names.size();

			const int points = 1;
			// We will have two waypoints in this trajectory
			trajectory.points.resize(points);

			// First trajectory point
			// Positions
            for(int ind = 0; ind < points; ind++) {

                trajectory.points[ind].positions.resize(num_joints);
                trajectory.points[ind].positions[0] = 0.0;
                trajectory.points[ind].positions[1] = 0.0;
                trajectory.points[ind].positions[2] = 1.0;

				trajectory.points[ind].velocities.resize(num_joints);
                trajectory.points[ind].velocities[0] =  0.0;
                trajectory.points[ind].velocities[1] =  0.0;
                trajectory.points[ind].velocities[2] =  0.0;

                trajectory.points[ind].accelerations.resize(num_joints);
                trajectory.points[ind].accelerations[0] =  0.0;
                trajectory.points[ind].accelerations[1] =  0.0;
                trajectory.points[ind].accelerations[2] =  0.0;

			    trajectory.points[ind].time_from_start = ros::Duration(2*ind+1);
            }

			//we are done; return the goal
			return trajectory;
		}
		
#if 0
		//! Returns the current state of the action
		actionlib::SimpleClientGoalState getState()
		{
			return traj_client_->getState();
		}
#endif
};


ros::ServiceClient spline_gen;
bool run(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*response*/)
{
	RobotBase base;

	base_trajectory::GenerateSpline srv;
	srv.request.points = base.genTrajectory().points;

	spline_gen.call(srv);

	ROS_WARN("run_test_driver");

	return true;
}

int main(int argc, char** argv)
{
	// Init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("/base_trajectory/driver_run", run);
	spline_gen = nh.serviceClient<base_trajectory::GenerateSpline>("/base_trajectory/spline_gen");

	ros::spin();

}
