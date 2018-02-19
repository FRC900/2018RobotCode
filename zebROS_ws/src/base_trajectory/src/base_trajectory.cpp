#include <time.h>
// Let's break C++
// Need to access a private variable from quintic_spline_segment
// by using a derived class. What could possibly go wrong?
#define private protected
#include <trajectory_interface/quintic_spline_segment.h>
#undef private
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <base_trajectory/profiler.h>
#include <boost/assign.hpp>
#include <pluginlib/class_list_macros.h>
// Define a member function to read spline coefficents
#include <base_trajectory/GenerateSwerveProfile.h>
#include <talon_swerve_drive_controller/MotionProfile.h> //TODO remove

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

std::shared_ptr<swerve_profile::swerve_profiler> profile_gen;

ros::Duration period;

ros::ServiceClient run_prof;
ros::ServiceClient graph_prof;

//ros::Duration period;
//ros::Publisher pub;

//const JointTrajectoryConstPtr& msg
//trajectory_msgs::JointTrajectory out_msg;

bool generate(base_trajectory::GenerateSwerveProfile::Request &msg,
base_trajectory::GenerateSwerveProfile::Response &out_msg
)
{
	ros::Time start = ros::Time::now();
	// Hold current position if trajectory is empty
	if (msg.joint_trajectory.points.empty())
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
	// rather than running linear mition
	for (size_t i = 0; i < joint_names.size(); i++)
		angle_wraparound.push_back(false);

	// Allocate memory to hold an
	// initial trajectory
	Trajectory hold_trajectory;
	typename Segment::State current_joint_state = typename Segment::State(1);
	for (unsigned int i = 0; i < n_joints; ++i)
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

	ros::Time init = ros::Time::now();
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
		trajectory = joint_trajectory_controller::initJointTrajectory<Trajectory>(msg.joint_trajectory, next_update_time, options);
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
	std::vector<swerve_profile::spline_coefs> x_splines, y_splines, orient_splines;

	std::vector<double> end_points;

	swerve_profile::spline_coefs temp_holder_s;	

	for (size_t seg = 0; seg < trajectory[0].size(); seg++)
	{
		
		std::cout << "joint = " << joint_names[0] << " seg = " << seg;
		std::cout << " start_time = " << trajectory[0][seg].startTime();
		std::cout << " end_time = " << trajectory[0][seg].endTime() << std::endl;
		auto coefs = trajectory[0][seg].getCoefs();
		
		std::cout << "coefs: " << coefs[0][0]<< " " << coefs[0][1]<< " " << coefs[0][2]<< " " << coefs[0][3]<< " " << coefs[0][4]<< " " << coefs[0][5];
		
		temp_holder_s.a = coefs[0][5]; 
		temp_holder_s.b = coefs[0][4]; 
		temp_holder_s.c = coefs[0][3]; 
		temp_holder_s.d = coefs[0][2]; 
		temp_holder_s.e = coefs[0][1]; 
		temp_holder_s.f = coefs[0][0]; 

		//a = coef[0][0]
		//b = coef[0][1]
		//c = coef[0][2]
		//d = coef[0][3]
		//e = coef[0][4]
		//f = coef[0][5]
		std::cout << std::endl;
		x_splines.push_back(temp_holder_s);
	
		end_points.push_back(trajectory[0][seg].endTime());
	}
	for (size_t seg = 0; seg < trajectory[1].size(); seg++)
	{
		
		std::cout << "joint = " << joint_names[1] << " seg = " << seg;
		std::cout << " start_time = " << trajectory[1][seg].startTime();
		std::cout << " end_time = " << trajectory[1][seg].endTime()<< std::endl;
		auto coefs = trajectory[1][seg].getCoefs();
		
		std::cout << "coefs: " << coefs[0][0]<< " " << coefs[0][1]<< " " << coefs[0][2]<< " " << coefs[0][3]<< " " << coefs[0][4]<< " " << coefs[0][5];
		

		temp_holder_s.a = coefs[0][5]; 
		temp_holder_s.b = coefs[0][4]; 
		temp_holder_s.c = coefs[0][3]; 
		temp_holder_s.d = coefs[0][2]; 
		temp_holder_s.e = coefs[0][1]; 
		temp_holder_s.f = coefs[0][0]; 

		//a = coef[0][0]
		//b = coef[0][1]
		//c = coef[0][2]
		//d = coef[0][3]
		//e = coef[0][4]
		//f = coef[0][5]
		std::cout << std::endl;
		y_splines.push_back(temp_holder_s);
	}
	for (size_t seg = 0; seg < trajectory[2].size(); seg++)
	{
		
		std::cout << "joint = " << joint_names[2] << " seg = " << seg;
		std::cout << " start_time = " << trajectory[2][seg].startTime();
		std::cout << " end_time = " << trajectory[2][seg].endTime()<< std::endl;
		auto coefs = trajectory[2][seg].getCoefs();
		
		std::cout << "coefs: " << coefs[0][0]<< " " << coefs[0][1]<< " " << coefs[0][2]<< " " << coefs[0][3]<< " " << coefs[0][4]<< " " << coefs[0][5];
		
		temp_holder_s.a = coefs[0][5]; 
		temp_holder_s.b = coefs[0][4]; 
		temp_holder_s.c = coefs[0][3]; 
		temp_holder_s.d = coefs[0][2]; 
		temp_holder_s.e = coefs[0][1]; 
		temp_holder_s.f = coefs[0][0]; 

		//a = coef[0][0]
		//b = coef[0][1]
		//c = coef[0][2]
		//d = coef[0][3]
		//e = coef[0][4]
		//f = coef[0][5]
		std::cout << std::endl;
		orient_splines.push_back(temp_holder_s);
	}

	ros::Time gen = ros::Time::now();
	std::cout << "Gen time = " << (gen - init).toSec() << std::endl;

	// Generate and publish output message
	// TODO : make this the results of the 
	// service
	out_msg.header.stamp = ros::Time::now();

	// Add names of the joints to the message
	for (auto it = joint_names.cbegin(); it != joint_names.cend(); ++it)
		out_msg.joint_names.push_back(*it);
	//out_msg.joint_names.push_back("steering_vel");	
	// Figure out how many points are going to be 
	// calculated along the  path. 
	// TODO :should be a simple divide, worry about
	// boundary conditions

	// Loop through and generate 1 point per timestep
	// each point has a pos/vel/acc data for each of the
	// three x,y,z paths
	// TODO : this takes the majority of the runtime
	//        it is also a prime target for multi-threading
	//        since each point is generated independently of
	//        the last.

	//TODO: we need to get radius and path angle out of the 
	//x_y spline, which will make this no longer be generalized, 
	//but that should be fine
	//We also need to generate points by arc length rather than by "time"
	

	//Radius = (x'^2 + y'^2)^(3/2) / (x' * y'' - y' * x'')
	//If the equations for joints y and x are: a_y t^5 + b_y t^4 + c_y t^3 + d_y t^2 + e_y t + f_y and a_x t^5 + b_x t^4 + c_x t^3 + d_x t^2 + e_x t + f_x
	
	//radius = ((5a_xt^4 + 4b_xt^3 + 3c_xt^2 + 2d_xt + e_x)^2 + (5a_yt^4 + 4b_yt^3 + 3c_yt^2 + 2d_yt + e_y)^2)^(3/2)/((5a_xt^4 + 4b_xt^3 + 3c_xt^2 + 2d_xt + e_x)*(20a_yt^3 + 12b_yt^2 + 6c_yt + 2d_y)-(5a_yt^4 + 4b_yt^3 + 3c_yt^2 + 2d_yt + e_y)*(20a_xt^3 + 12b_xt^2 + 6c_xt + 2d_x))

	//path_angle = atan2(-(5a_xt^4 + 4b_xt^3 + 3c_xt^2 + 2d_xt + e_x), 5a_yt^4 + 4b_yt^3 + 3c_yt^2 + 2d_yt + e_y)
	//TODO Something with below?
	//ros::Time pub_time = ros::Time::now();
	//std::cout << "Pub time = " << (pub_time - gen).toSec() << std::endl;
	
	
	
	//TODO: Harvest splines
	



	if(profile_gen->generate_profile(x_splines, y_splines, orient_splines, msg.initial_v, msg.final_v, out_msg, end_points))
	{
		ROS_INFO("SUCCESS - NICEEEE");
		//TODO: remove below code
		
		talon_swerve_drive_controller::MotionProfile srv_msg;
		srv_msg.request.joint_trajectory.header = out_msg.header;	
		srv_msg.request.joint_trajectory.joint_names = out_msg.joint_names;	
		srv_msg.request.joint_trajectory.points = out_msg.points;
		graph_prof.call(srv_msg);
		
		return true;
	}
	else
	{
		ROS_WARN("FAILED - TRAGIC");
		return false;
	}
	//TODO parametrize by arc length in a better way:
	
	
	/*
	for (ros::Duration now(0); now <= end_time; now += period, point_count += 1)
	{
		out_msg.points[point_count].time_from_start = now;
		for (size_t i = 0; i < joint_names.size(); ++i)
		{
			typename Segment::State desired_state;
			typename TrajectoryPerJoint::const_iterator segment_it = sample(trajectory[i], now.toSec(), desired_state);
			if (trajectory[i].end() == segment_it)
			{
				// Non-realtime safe, but should never happen under normal operation
				ROS_ERROR("Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
				return;
			}
			out_msg.points[point_count].positions.push_back(desired_state.position[0]);
			out_msg.points[point_count].velocities.push_back(desired_state.velocity[0]);
			out_msg.points[point_count].accelerations.push_back(desired_state.acceleration[0]);
		}
	}
	*/

	

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;


	double loop_hz;

	nh.param<double>("loop_hz", loop_hz, 50.);


	//TODO: make below read from config file or something

	profile_gen = std::make_shared<swerve_profile::swerve_profiler>(.425, 9.0, 3.0, 6.0, 7.0, 1/loop_hz);

	ros::ServiceServer service = nh.advertiseService("/base_trajectory/command", generate);

	//
	run_prof = nh.serviceClient<talon_swerve_drive_controller::MotionProfile>("/frcrobot/swerve_drive_controller/run_profile");
	graph_prof = nh.serviceClient<talon_swerve_drive_controller::MotionProfile>("visualize_profile");
	ros::spin();
}
