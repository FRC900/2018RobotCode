#pragma once

#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <talon_controllers/talon_controller_interface.h>
#include <talon_controllers/CloseLoopControllerMsg.h>
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

		virtual bool init(hardware_interface::TalonCommandInterface* hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw, n))
				return false;

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &TalonController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time& /*time*/)
		{
			// Start controller with motor stopped
			// for great safety
			command_buffer_.writeFromNonRT(0.0);
		}
		virtual void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
		{
			talon_if_.setCommand(*command_buffer_.readFromRT());
		}


	private:
		TALON_IF talon_if_;
		ros::Subscriber sub_command_;
		realtime_tools::RealtimeBuffer<double> command_buffer_;
		void commandCB(const std_msgs::Float64ConstPtr& msg)
		{
			command_buffer_.writeFromNonRT(msg->data);
		}
};

class TalonPercentVbusController: public TalonController<TalonPercentVbusControllerInterface> 
{
	// Override or add methods different from the base class here
};

// Generic Close Loop controller. Allow users to specify 
// slot for PID values plus command per message. Trust lower
// level code will prevent repeatedly switching PID slot if
// it doesn't actually change from message to message.
class CloseLoopCommand
{
	public:
		CloseLoopCommand(void) :
			config_slot_(0),
			command_(0.0)
	{
	}
		CloseLoopCommand(int config_slot, double command) :
			config_slot_(config_slot),
			command_(command)
	{
	}

	int config_slot_;
	double command_;
};

template <class TALON_IF>
class TalonCloseLoopController :
public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:
		TalonCloseLoopController() {}
		~TalonCloseLoopController() {sub_command_.shutdown();}

		virtual bool init(hardware_interface::TalonCommandInterface* hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw, n))
				return false;

			sub_command_ = n.subscribe<CloseLoopControllerMsg>("command", 1, &TalonCloseLoopController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time& /*time*/)
		{
			// Start controller with motor stopped
			// for great safety
			command_buffer_.writeFromNonRT(CloseLoopCommand());
		}
		virtual void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
		{
			// Write both PID config slot and 
			// output to talon interface
			CloseLoopCommand cmd = *command_buffer_.readFromRT();
			talon_if_.setPIDConfig(cmd.config_slot_);
			talon_if_.setCommand(cmd.command_);
		}


	private:
		TALON_IF talon_if_;
		ros::Subscriber sub_command_;
		realtime_tools::RealtimeBuffer<CloseLoopCommand> command_buffer_;
		void commandCB(const CloseLoopControllerMsgConstPtr& msg)
		{
			command_buffer_.writeFromNonRT(CloseLoopCommand(msg->config_slot, msg->command));
		}
};

class TalonPositionCloseLoopController: public TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>
{
	// Override or add methods here
};

} // end namespace
