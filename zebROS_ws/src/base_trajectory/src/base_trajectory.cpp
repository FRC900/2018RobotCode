#include <time.h>
// Let's break C++!
// Need to access a private variable from quintic_spline_segment
// by using a derived class. What could possibly go wrong?
#define private protected
#include <trajectory_interface/quintic_spline_segment.h>
#undef private
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <boost/assign.hpp>
#include <base_trajectory/GenerateSpline.h>

namespace trajectory_interface
{
template<class ScalarType>
class MyQuinticSplineSegment: public QuinticSplineSegment<ScalarType>
{
	public: 
		std::vector<typename QuinticSplineSegment<ScalarType>::SplineCoefficients> getCoefs(void) const
		{
			return this->coefs_;
		}
};
}

typedef joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::MyQuinticSplineSegment<double>> Segment;

typedef std::vector<Segment> TrajectoryPerJoint;
typedef std::vector<TrajectoryPerJoint> Trajectory;

typedef trajectory_msgs::JointTrajectory::ConstPtr JointTrajectoryConstPtr;

ros::Duration period;

// input should be JointTrajectory[] custom message
// Output wil be array of spline coefficents swerve_point_generator/Coefs[] for x, y, orientation
bool generate(base_trajectory::GenerateSpline::Request &msg,
			  base_trajectory::GenerateSpline::Response &out_msg)
{
	const ros::Time start = ros::Time::now();
	// Hold current position if trajectory is empty
	if (msg.points.empty())
	{
		ROS_DEBUG("Empty trajectory command, stopping.");
		return false;
	}

	// Hard code 3 dimensions for paths to
	// follow - x&y translation and z rotation
	std::vector<std::string> joint_names = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	const size_t n_joints = joint_names.size();
	std::vector<bool> angle_wraparound;

	// Assume the path starts at time 0
	ros::Time next_update_time = ros::Time(0);
	ros::Time next_update_uptime = next_update_time;

	// Set this to false to prevent the code
	// from thinking we're driving rotation joints
	// rather than running linear motion
	for (size_t i = 0; i < n_joints; i++)
		angle_wraparound.push_back(false);

	// Allocate memory to hold an initial trajectory
	Trajectory hold_trajectory;
	typename Segment::State current_joint_state = typename Segment::State(1);
	for (size_t i = 0; i < n_joints; ++i)
	{
		current_joint_state.position[0] = 0;
		current_joint_state.velocity[0] = 0;
		Segment hold_segment(0.0, current_joint_state, 0.0, current_joint_state);

		TrajectoryPerJoint joint_segment;
		joint_segment.resize(1, hold_segment);
		hold_trajectory.push_back(joint_segment);
	}

	// This generates a starting trajectory
	// with the robot sitting still at location 0,0,0.
	// It is needed as an initial condition for the
	// robot to connect it to the first waypoint
	//
	// TODO : make the starting position and 
	// velocity a variable passed in to the 
	// path generation request.
	

	//TODO: WHAT BE THIS BRACKET
	{
		typename Segment::State hold_start_state = typename Segment::State(1);
		typename Segment::State hold_end_state = typename Segment::State(1);

		double stop_trajectory_duration = period.toSec() / 2.;
		const typename Segment::Time start_time  = 0;
		const typename Segment::Time end_time    = stop_trajectory_duration;
		const typename Segment::Time end_time_2x = 2.0 * stop_trajectory_duration;

		// Create segment that goes from current (pos,vel) to (pos,-vel)
		for (size_t i = 0; i < n_joints; ++i)
		{
			hold_start_state.position[0]     = 0.0;
			hold_start_state.velocity[0]     = 0.0;
			hold_start_state.acceleration[0] = 0.0;

			hold_end_state.position[0]       = 0.0;
			hold_end_state.velocity[0]       = -0.0;
			hold_end_state.acceleration[0]   = 0.0;

			hold_trajectory[i].front().init(start_time, hold_start_state, end_time_2x, hold_end_state);

			// Sample segment at its midpoint, that should have zero velocity
			hold_trajectory[i].front().sample(end_time, hold_end_state);

			// Now create segment that goes from current state to one with zero end velocity
			hold_trajectory[i].front().init(start_time, hold_start_state, end_time, hold_end_state);
		}
	}

	const ros::Time init = ros::Time::now();
	std::cout << "Init time = " << (init - start).toSec() << std::endl;

	// Set basic options for the trajectory
	// generation controller
	joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> options;
	options.other_time_base           = &next_update_uptime;
	options.current_trajectory        = &hold_trajectory;
	options.joint_names               = &joint_names;
	options.angle_wraparound          = &angle_wraparound;
	options.rt_goal_handle            = NULL;
	options.default_tolerances        = NULL;
	options.allow_partial_joints_goal = true;

	// Actually generate the new trajectory
	// This will create spline coefficents for
	// each of the x,y,z paths
	// TODO : take input from a service rather
	// than a topic
	Trajectory trajectory;
	try
	{
		trajectory_msgs::JointTrajectory jtm;
		jtm.joint_names = joint_names;
		jtm.points = msg.points;
		trajectory = joint_trajectory_controller::initJointTrajectory<Trajectory>(jtm, next_update_time, options);
		if (trajectory.empty())
		{
			ROS_WARN("Not publishing empty trajectory");
			return false;
		}
	}
	catch(const std::invalid_argument& ex)
	{
		ROS_ERROR_STREAM(ex.what());
		return false;
	}
	catch(...)
	{
		ROS_ERROR("Unexpected exception caught when initializing trajectory from ROS message data.");
		return false;
	}

	out_msg.orient_coefs.resize(trajectory[0].size());
	out_msg.x_coefs.resize(trajectory[0].size());
	out_msg.y_coefs.resize(trajectory[0].size());

	for (size_t seg = 0; seg < trajectory[0].size(); seg++)
	{
		for (size_t joint = 0; joint < n_joints; joint++)
		{
			std::cout << "joint = " << joint_names[joint] << " seg = " << seg;
			std::cout << " start_time = " << trajectory[joint][seg].startTime();
			std::cout << " end_time = " << trajectory[joint][seg].endTime() << std::endl;
			auto coefs = trajectory[joint][seg].getCoefs();

			std::cout << "coefs: " << coefs[0][0] << " " << coefs[0][1] << " " << coefs[0][2] << " " << coefs[0][3] << " " << coefs[0][4] << " " << coefs[0][5];

			std::vector<double> *m;

			if (joint == 0)
				m = &out_msg.x_coefs[seg].spline;
			else if (joint == 1)
				m = &out_msg.y_coefs[seg].spline;
			else if (joint == 2)
				m = &out_msg.orient_coefs[seg].spline;
			else
			{
				ROS_WARN("Unexpected joint number constructing out_msg in base_trajectory");
				continue;
			}

			// Push in reverse order to match expectations
			// of point_gen code?
			m->clear();
			for (int i = 0; i <= 5; i++)
				m->push_back(coefs[0][i]);

			std::cout << std::endl;
		}
		// All splines in a waypoint end at the same time?
		out_msg.end_points.push_back(trajectory[0][seg].endTime());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;

	double loop_hz;

	nh.param<double>("loop_hz", loop_hz, 50.);
	period = ros::Duration(1.0 / loop_hz);

	ros::ServiceServer service = nh.advertiseService("/base_trajectory/spline_gen", generate);

	ros::spin();
}
