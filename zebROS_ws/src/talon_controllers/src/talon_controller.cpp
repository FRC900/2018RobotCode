#include <talon_controllers/talon_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(talon_controllers::TalonPercentVbusController,
		               controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talon_controllers::TalonFollowerController,
		               controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talon_controllers::TalonPositionCloseLoopController,
		               controller_interface::ControllerBase)
