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
			talon_ = hw->getHandle(joint_name);
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &TalonController::commandCB, this);
			return true;
		}

		void starting(const ros::Time& /*time*/);
		void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) 
		{
			talon_.setCommand(*command_buffer_.readFromRT());
		}

		hardware_interface::TalonCommandHandle talon_;
		realtime_tools::RealtimeBuffer<double> command_buffer_;

	private:
		ros::Subscriber sub_command_;
		void commandCB(const std_msgs::Float64ConstPtr& msg)
	   	{
			command_buffer_.writeFromNonRT(msg->data);
		}
};

}
