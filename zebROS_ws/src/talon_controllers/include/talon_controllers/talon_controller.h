#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <talon_controllers/talon_controller_interface.h>
#include <talon_controllers/CloseLoopControllerMsg.h>

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
		~TalonController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw, nullptr, n))
				return false;

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &TalonController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/)
		{
			// Start controller with motor stopped
			// for great safety
			command_buffer_.writeFromNonRT(0.0);
		}
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
		{
			// Take the most recent value stored in the command
			// buffer (the most recent value read from the "command"
			// topic) and set the Talon to that commanded value
			talon_if_.setCommand(*command_buffer_.readFromRT());
		}

	private:
		TALON_IF talon_if_;
		ros::Subscriber sub_command_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.  This buffer is read in each call to update()
		// to get the command to send to the Talon
		realtime_tools::RealtimeBuffer<double> command_buffer_;

		// Take each message read from the "command" topic and push
		// it into the command buffer. This buffer will later be
		// read by update() and sent to the Talon.  The buffer
		// is used because incoming messages aren't necessarily
		// synchronized to update() calls - the buffer holds the
		// most recent command value that update() can use
		// when the update() code is run.
		void commandCB(const std_msgs::Float64ConstPtr &msg)
		{
			command_buffer_.writeFromNonRT(msg->data);
		}
};

class TalonPercentOutputController: public TalonController<TalonPercentOutputControllerInterface>
{
		// Override or add methods different from the base class here
};

// Generic Close Loop controller. Allow users to specify
// slot for PID values plus command per message. Trust lower
// level code will prevent repeatedly switching PID slot if
// it doesn't actually change from message to message.
//RG: In talon controller interface current closed loop control also inherits closed loop stuff
//This command involves PID so maybe current control shouldn't be considered closed loop control?
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
		~TalonCloseLoopController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw, nullptr, n))
				return false;

			sub_command_ = n.subscribe<CloseLoopControllerMsg>("command", 1, &TalonCloseLoopController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/)
		{
			// Start controller with motor stopped
			// for great safety
			command_buffer_.writeFromNonRT(CloseLoopCommand());
		}
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
		{
			// Write both PID config slot and
			// output to talon interface
			CloseLoopCommand cmd = *command_buffer_.readFromRT();
			talon_if_.setPIDFSlot(cmd.config_slot_);
			talon_if_.setCommand(cmd.command_);
		}

	protected:
		TALON_IF talon_if_;
		ros::Subscriber sub_command_;
		realtime_tools::RealtimeBuffer<CloseLoopCommand> command_buffer_;
		void commandCB(const CloseLoopControllerMsgConstPtr &msg)
		{
			command_buffer_.writeFromNonRT(CloseLoopCommand(msg->config_slot, msg->command));
		}
};

class TalonPositionCloseLoopController: public TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>
{
		// Override or add methods here
};
class TalonVelocityCloseLoopController: public TalonCloseLoopController<TalonVelocityCloseLoopControllerInterface>
{
		// Override or add methods here
};

// Follower controller sets up a Talon to mirror the actions
// of another talon. This talon is defined by joint name in
// params/yaml config.
class TalonFollowerController:
	public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface,
	hardware_interface::TalonStateInterface>
{
	public:
		TalonFollowerController() {}
		~TalonFollowerController() {}

		bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw->get<hardware_interface::TalonCommandInterface>(),
										hw->get<hardware_interface::TalonStateInterface>(), n))
				return false;

			return true;
		}

		void starting(const ros::Time & /*time*/)
		{
		}
		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
		{
		}

	private:
		// Keep ownership of the Talon being run in follower mode.
		// Even though there's currently no commands that can be sent
		// to the Talon keeping this will prevent other controllers
		// from grabbing that Talon until this controller is
		// explicitly unloaded.
		TalonFollowerControllerInterface talon_if_;
};

// Convert Linear Position and Displacement to radians
// and input on the Talon
// Use of Positional PID and the radius of the gear next
// to the Talon
class TalonLinearPositionCloseLoopController :
	public TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>
{	
	//Used radius	
	private:
		double radius_;
	public:
		
		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>::init(hw,n))
				return false;

			//radius for length
			n.getParam("radius", radius_);
		}

		// Same as TalonClosedLoopController but setCommand
		// has converted radians as input
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Write both PID config slot and
			// output to talon interface
			CloseLoopCommand cmd = *command_buffer_.readFromRT();
			talon_if_.setPIDFSlot(cmd.config_slot_);
			talon_if_.setCommand(cmd.command_ / radius_);
		}
};

}
