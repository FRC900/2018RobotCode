#pragma once
#include <joint_trajectory_controller/hardware_interface_adapter.h>
/**
 * \brief Adapter for a hardware interface that isn't really a hardware interface. 
 * Instead publishes positions, velocities, accelerations as a message.
 * Use JointCommandInterface as a type simple to make it load correctly
 * in the code but the JCI is never actually used.
 *
 * The following is an example configuration of a controller that uses this adapter.
 * \code
 * head_controller:
 *   type: "position_controllers/JointCommandInterface"
 *   joints:
 *     - x_linear_joint
 *     - y_linear_joint
 *     - z_rotation_joint
 *   
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     x_linear_joint: {trajectory: 0.05, goal: 0.02}
 *     y_linear_joint: {trajectory: 0.05, goal: 0.02}
 *     z_rotation_joint: {trajectory: 0.05, goal 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */

// Hack - use a generic JointCommandInterface as the type for
// our realtime publisher of data

template <class State>
class HardwareInterfaceAdapter<hardware_interface::JointCommandInterface, State>
{
	public:
		HardwareInterfaceAdapter(void) 
		{}

		bool init(std::vector<hardware_interface::JointHandle>& /*joint_handles*/, ros::NodeHandle& /*controller_nh*/)
		{
			return true;
		}

		void starting(const ros::Time& /*time*/)
		{
			// Establish zero?
		}

		void stopping(const ros::Time& /*time*/) {}

		void updateCommand(const ros::Time&     /*time*/,
				const ros::Duration& /*period*/,
				const State&         /*desired_state*/,
				const State&         /*state_error*/)
		{
		}
};

