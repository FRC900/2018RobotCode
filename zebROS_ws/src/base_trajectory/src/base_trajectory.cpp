#include <time.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <base_trajectory/hardware_interface_adapter.h>

static constexpr double BILLION = 1000000000.0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;
	double loop_hz;
	joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>, hardware_interface::JointCommandInterface> jtc;

	nh.param<double>("loop_hz", loop_hz, 50);

	hardware_interface::RobotHW robot_hw;
	double foo;
	const std::vector<std::string> joint_names = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	hardware_interface::JointCommandInterface command_if;
	for (auto joint_name : joint_names)
	{
		hardware_interface::JointStateHandle sh(joint_name, &foo, &foo, &foo);

		hardware_interface::JointHandle jh(sh, &foo);
		command_if.registerHandle(jh);
	}
	robot_hw.registerInterface(&command_if);
	controller_interface::ControllerBase::ClaimedResources claimed_resources; // Gets populated during initRequest call

	controller_interface::ControllerBase *cbp = &jtc;
	cbp->initRequest(&robot_hw, nh, nh, claimed_resources);

	jtc.startRequest(ros::Time::now());

	struct timespec last_time;
	clock_gettime(CLOCK_MONOTONIC, &last_time);

	ros::Rate rate(loop_hz);
	while(ros::ok())
	{
		struct timespec current_time;
		clock_gettime(CLOCK_MONOTONIC, &current_time);

		const ros::Duration elapsed_time(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
		last_time = current_time;

		jtc.update(ros::Time::now(), elapsed_time);
		ros::spinOnce();
		rate.sleep();
	}

	jtc.stopping(ros::Time::now());
}
