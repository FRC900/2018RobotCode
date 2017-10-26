#pragma once

#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <talon_controllers/talon_controller_interface.h>
#include <realtime_tools/realtime_buffer.h>

namespace talon_controllers
{

/**
 * \brief Simple Talon Controllers`
 *
 * These classes implement simple controllers for TalonSRX
 * hardware running in various modes.
 *
 * \section ROS interface
 *
 * \param type Must be "Talon<type>Controller".
 * \param joint Name of the talon-controlled joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint setpoint to apply
 */


// Since most controllers are going to share a lot of common code,
// create a base class template. The big difference between controllers
// will be the mode the Talon is run in. This is specificed by the type
// of talon interface, so make this the templated parameter.
template <class TALON_IF>
class TalonController: 
public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:
		TalonController() {}
		~TalonController() {sub_command_.shutdown();}

		bool init(hardware_interface::TalonCommandInterface* hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw, n))
				return false;

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &TalonController::commandCB, this);
			return true;
		}

		void starting(const ros::Time& /*time*/)
		{
			// Start controller with motor stopped
			// for great safety
			command_buffer_.writeFromNonRT(0.0);
		}
		void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
		{
			talon_if_.setCommand(*command_buffer_.readFromRT());
		}

		realtime_tools::RealtimeBuffer<double> command_buffer_;

	private:
		TALON_IF talon_if_;
		ros::Subscriber sub_command_;
		void commandCB(const std_msgs::Float64ConstPtr& msg)
		{
			command_buffer_.writeFromNonRT(msg->data);
		}
};

class TalonPercentVbusController: public TalonController<TalonPercentVbusControllerInterface> 
{
	// Override or add methods different from the base class here
};

// TODO : maybe make a generic CloseLoop class with a setPIDFSlot method
// and then derive both Position and Velocity CloseLoop classes from them?
// This would keep us from having to write that method in both of
// the final classes
class TalonPositionCloseLoopController: public TalonController<TalonPositionCloseLoopControllerInterface>
{
	// Override or add methods different from the base class here
};

} // end namespace
