// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <base_trajectory/hardware_interface_adapter.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

namespace base_trajectory_controllers
{
	/**
	 * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and publishes state without sending commands to an interface
	 */
	typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
			hardware_interface::JointCommandInterface>
				JointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(base_trajectory_controllers::JointTrajectoryController, controller_interface::ControllerBase)
