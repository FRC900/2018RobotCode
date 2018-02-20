#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <base_trajectory/GenerateSwerveProfile.h>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>
#include <talon_swerve_drive_controller/FullGen.h>

ros::ServiceClient point_gen;
ros::ServiceClient swerve_control;

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
		trajectory_msgs::JointTrajectory genTrajectory(void) const
		{
			//our goal variable
			trajectory_msgs::JointTrajectory trajectory;

			// First, the joint names, which apply to all waypoints
			trajectory.joint_names.push_back("x_linear_joint");
			trajectory.joint_names.push_back("y_linear_joint");
			trajectory.joint_names.push_back("z_rotation_joint");

			const size_t num_joints = trajectory.joint_names.size();

			// We will have two waypoints in this trajectory
			trajectory.points.resize(2);

			// First trajectory point
			// Positions
			int ind = 0;
			trajectory.points[ind].positions.resize(num_joints);
			trajectory.points[ind].positions[0] =  2.0;
			trajectory.points[ind].positions[1] =  0.0;
			trajectory.points[ind].positions[2] =  0.0;
			// Velocities
			trajectory.points[ind].velocities.resize(num_joints);
			trajectory.points[ind].velocities[0] =  0.0;
			trajectory.points[ind].velocities[1] =  0.0;
			trajectory.points[ind].velocities[2] =  0.0;

			// To be reached 1 second after starting along the trajectory
			trajectory.points[ind].time_from_start = ros::Duration(4.0);

			// Second trajectory point
			// Positions
			ind += 1;
			trajectory.points[ind].positions.resize(num_joints);
			trajectory.points[ind].positions[0] = 3.0;
			trajectory.points[ind].positions[1] = 0.1;
			trajectory.points[ind].positions[2] = 10.0;
			// Velocities
			trajectory.points[ind].velocities.resize(num_joints);
			trajectory.points[ind].velocities[0] =  0.0;
			trajectory.points[ind].velocities[1] =  0.0;
			trajectory.points[ind].velocities[2] =  0.0;

			// To be reached 2 seconds after starting along the trajectory
			trajectory.points[ind].time_from_start = ros::Duration(8.0);

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
RobotBase base;
bool run(talon_swerve_drive_controller::FullGen::Request &msg,
talon_swerve_drive_controller::FullGen::Response &out_msg)
{

	talon_swerve_drive_controller::FullGen srv;
	srv.request.joint_trajectory = base.genTrajectory();
	srv.request.initial_v = 0.0;
	srv.request.final_v = 0.0;
	point_gen.call(srv);
	out_msg.points = srv.response.points;

	 ROS_WARN("run_test_driver");

	talon_swerve_drive_controller::MotionProfilePoints srv_msg_points;

	srv_msg_points.request.dt = srv.response.dt;	
	srv_msg_points.request.points = srv.response.points;	
	srv_msg_points.request.buffer = true;	
	srv_msg_points.request.mode = false;
	srv_msg_points.request.run = false;

	swerve_control.call(srv_msg_points);
		
	return true;

}

int main(int argc, char** argv)
{
	// Init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;


	point_gen = nh.serviceClient<talon_swerve_drive_controller::FullGen>("/point_gen/command");
	swerve_control = nh.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile");

	//pub = nh.serviceClient<base_trajectory::GenerateSwerveProfile>("/base_trajectory/command");
	// Start the trajectory
	//
        ros::ServiceServer service = nh.advertiseService("/base_trajectory/driver_run", run);

	ros::spin();

}
