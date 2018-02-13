#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>


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
		RobotBase()
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
			trajectory.points[ind].positions[1] =  1.0;
			trajectory.points[ind].positions[2] = -1.0;
			// Velocities
			for (size_t j = 0; j < num_joints; ++j)
				trajectory.points[ind].velocities.push_back(0);

			// To be reached 1 second after starting along the trajectory
			trajectory.points[ind].time_from_start = ros::Duration(4.0);

			// Second trajectory point
			// Positions
			ind += 1;
			trajectory.points[ind].positions.resize(num_joints);
			trajectory.points[ind].positions[0] = -3.;
			trajectory.points[ind].positions[1] = 2;
			trajectory.points[ind].positions[2] = -1.9;
			// Velocities
			for (size_t j = 0; j < num_joints; ++j)
				trajectory.points[ind].velocities.push_back(0);

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

int main(int argc, char** argv)
{
	// Init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	RobotBase base;

	ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("command",1, true);
	// Start the trajectory
	//
	ros::Rate rate (20);
	while (pub.getNumSubscribers() != 2)
		rate.sleep();
	
	pub.publish(base.genTrajectory());
		rate.sleep();
}
