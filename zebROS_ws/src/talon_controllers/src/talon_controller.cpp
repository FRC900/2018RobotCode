#include <talon_controllers/talon_controller.h>
#include <pluginlib/class_list_macros.h>

void talon_controllers::TalonController::starting(const ros::Time& /*time*/)
{
  // Start controller with 0.0 velocity
  command_buffer_.writeFromNonRT(0.0);
}


PLUGINLIB_EXPORT_CLASS(talon_controllers::TalonController,controller_interface::ControllerBase)
