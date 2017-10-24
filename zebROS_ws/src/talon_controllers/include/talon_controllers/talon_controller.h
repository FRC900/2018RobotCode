#pragma once

#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <talon_interface/talon_interface.h>
#include <realtime_tools/realtime_buffer.h>

namespace talon_controllers
{

/**
 * \brief Simple Talon Controller
 *
 * This class is a controller used for testing Talon implementation
 *
 * \section ROS interface
 *
 * \param type Must be "TalonController".
 * \param joint Name of the talon-controlled joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint setpoint to apply
 */

	// Create a wrapper class for each Talon mode.  For the basic controllers
	// this isn't really helpful since the code could just be included in the
	// controller itself.  But consider a more complex controller, for example
	// swerve. The swerve controller runs a number of wheels, and each wheel
	// has both position and velocity.  The wheel class might create a 
	// TalonPosisionPIDControllerInterface member var for the position moter and 
	// also a TalonVelocityPIDControllerInterface member var for the velocity.  
	// And since it will be creating one per wheel, it makes sense to wrap the common
	// init code into a class rather than duplicate it for each wheel. Another
	// controller - say a shooter wheel - could also use this same code to 
	// create a talon handle to access that motor
class TalonControllerInterface
{
	public:
		virtual bool init(hardware_interface::TalonCommandInterface* hw, const std::string &name)
		{
			talon_ = hw->getHandle(name);
			talon_.setCommand(0);
			// perform any other generic initializtion here
			return true;
		}
		virtual void setCommand(const double command)
		{
			talon_.setCommand(command);
		}
	protected:
		hardware_interface::TalonCommandHandle talon_;
};

class TalonPercentVbusControllerInterface : public TalonControllerInterface
{
	public:
		bool init(hardware_interface::TalonCommandInterface* hw, const std::string &name)
		{
			if (!TalonControllerInterface::init(hw, name))
				return false;
			talon_.setMode(hardware_interface::TalonMode_PercentVbus);
			// perform any generic initializtion here
			return true;
		}
};

// Maybe make a TalonPIDControllerInterface instead, and then
// derive a PositionPID and VeloctyPID from that?
class TalonPositionPIDControllerInterface : public TalonControllerInterface
{
	public:
		bool init(hardware_interface::TalonCommandInterface* hw, const std::string &name,
				const double p[2], const double i[2], const double d[2],
			   	const double f[2], const double i_zone[2])
		{
			// Call base class init for common setup code
			if (!TalonControllerInterface::init(hw, name))
				return false;
			// perform any PositionPID specific 
			// initializtion here
			//
			talon_.setMode(hardware_interface::TalonMode_PositionCloseLoop);
			// for (int slot = 0; slot < 2; slot++)
			// {
			// talon_setPgain(slot, p[slot]); 
			// // and I, D, F, Izone
			// }
			return true;
		}
		bool init(hardware_interface::TalonCommandInterface* hw, const std::string &name,
				const double p, const double i, const double d,
			   	const double f, const double i_zone)
		{
			double this_p[2] = {p, p};
			double this_i[2] = {i, i};
			double this_d[2] = {d, d};
			double this_f[2] = {f, f};
			double this_izone[2] = {i_zone, i_zone};
			return init(hw, name, this_p, this_i, this_d, this_f, this_izone);
		}
};

// Really should call this TalonPercentVbusController instead
class TalonController: 
public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:
		TalonController() {}
		~TalonController() {sub_command_.shutdown();}

		bool init(hardware_interface::TalonCommandInterface* hw, ros::NodeHandle &n)
		{
			std::string joint_name;
			if (!n.getParam("joint", joint_name))
			{
				ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
				return false;
			}
			talon_.init(hw, joint_name);
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &TalonController::commandCB, this);
			return true;
		}

		void starting(const ros::Time& /*time*/);
		void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) 
		{
			talon_.setCommand(*command_buffer_.readFromRT());
		}

		realtime_tools::RealtimeBuffer<double> command_buffer_;

	private:
		TalonPercentVbusControllerInterface talon_;
		ros::Subscriber sub_command_;
		void commandCB(const std_msgs::Float64ConstPtr& msg)
	   	{
			command_buffer_.writeFromNonRT(msg->data);
		}
};

}
