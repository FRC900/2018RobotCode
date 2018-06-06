#include "mech_controller/mech_controller.h"

namespace mech_controller
{
bool MechController::init(hardware_interface::RobotHW *hw,
							ros::NodeHandle			&root_nh,
							ros::NodeHandle			&controller_nh)
{
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
	//if I had to read values from fake joints (like line break sensors) I would initialize a JointStateInterface, then getHandle
	//if I had to change non-Talon joint values (like pneumatics) I would initialize a PositionJointInterface, then getHandle

	//init the joint with the tci, tsi (not used), the node handle, and dynamic reconfigure (t/f)
	if (!mech_joint_.initWithNode(talon_command_iface, nullptr, controller_nh))
	{
		ROS_ERROR("Cannot initialize mech_joint!");
		return false;
	}

	//why don't you need a handle for Talon joints?

	//set soft limits, deadband, neutral mode, PIDF slots, acceleration and cruise velocity, all the things HERE

	/*mech_joint_.setPIDFSlot(0);
	mech_joint_.setMotionAcceleration(1); //TODO
	mech_joint_.setMotionCruiseVelocity(1); //TODO*/

	sub_command_ = controller_nh.subscribe("mech_pos", 1, &MechController::mechPosCallback, this); //input topic to subscribe to, queue size, callback (void not bool!) and the MechController itself
	service_command_ = controller_nh.advertiseService("mech_posS", &MechController::cmdService, this);

	return true;
}

void MechController::starting(const ros::Time &time) {
	ROS_ERROR_STREAM("MechController was started");
}

void MechController::update(const ros::Time &time, const ros::Duration &period) {
	float curr_cmd = *(command_.readFromRT());
	ROS_ERROR_STREAM("curr_cmd : " << curr_cmd);
	mech_joint_.setCommand(curr_cmd);
}

void MechController::stopping(const ros::Time &time) {
}

bool MechController::cmdService(mech_controller::SetPosition::Request &req, mech_controller::SetPosition::Response &/*response*/) {
	if(isRunning())
		command_.writeFromNonRT(req.position);
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
		return false;
	}
	return true;
}

void MechController::mechPosCallback(const std_msgs::Float64 &command) {
	if(isRunning())
		command_.writeFromNonRT(command.data);
	else
		ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS( mech_controller::MechController, controller_interface::ControllerBase)
