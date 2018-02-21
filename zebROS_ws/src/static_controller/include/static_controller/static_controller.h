#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace static_controller
{

/**
 * \brief Static controller
 *
 * This class creates a controller which sets a joint to
 * a fixed value when started. Used mainly to communicate 
 * state to the controller manager. 
 *
 * \section ROS interface
 *
 * \param joint Name of the joint to set
 * \param value value to set when starting the controller
 *
 */

class StaticController:
	public controller_interface::Controller<hardware_interface::JointCommandInterface>
{
	public:
		StaticController(void) {}
		~StaticController() {}

		virtual bool init(hardware_interface::JointCommandInterface *hw, ros::NodeHandle &n) override;

		virtual void starting(const ros::Time & /*time*/) override;
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override;

	private:
		hardware_interface::JointHandle handle_;
		double value_;
};
}
