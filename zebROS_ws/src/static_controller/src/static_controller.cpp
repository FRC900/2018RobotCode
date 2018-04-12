#include <ros/node_handle.h>
#include <static_controller/static_controller.h>
#include <pluginlib/class_list_macros.h>

namespace static_controller
{
// Implement a simple controller. The only job it has is to write a value
// to a given joint when started.  We load this controller last to indicate that
// all the other controllers have been loaded and the robot is now ready
// for competition
bool StaticController::init(hardware_interface::JointCommandInterface *hw, ros::NodeHandle &n)
{
	std::string joint_name;
	if (!n.getParam("joint", joint_name))
	{
		ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
		return false;
	}
	if (!n.getParam("value", value_))
	{
		ROS_ERROR("No value given (namespace: %s)", n.getNamespace().c_str());
		return false;
	}
	handle_ = hw->getHandle(joint_name);
	return true;
}

void StaticController::starting(const ros::Time& /*time*/)
{
	handle_.setCommand(value_);
}

void StaticController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
}

}

#if 0
void static_controller::update()
{
}
#endif

PLUGINLIB_EXPORT_CLASS(static_controller::StaticController,controller_interface::ControllerBase)
