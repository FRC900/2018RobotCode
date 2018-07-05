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
	if (!joint_1.initWithNode(talon_command_iface, nullptr, controller_nh))
	{
		ROS_ERROR("Cannot initialize joint 1!");
		return false;
	}

	if (!joint_2.initWithNode(talon_command_iface, nullptr, controller_nh))
	{
		ROS_ERROR("Cannot initialize joint 2!");
		return false;
	}

	//set soft limits, deadband, neutral mode, PIDF slots, acceleration and cruise velocity, all the things HERE

	/*joint_1.setPIDFSlot(0);
	joint_1.setMotionAcceleration(1); //TODO
	joint_1.setMotionCruiseVelocity(1); //TODO*/

	sub_command_ = controller_nh.subscribe("mech_pos", 1, &MechController::mechPosCallback, this); //input topic to subscribe to, queue size, callback (void not bool!) and the MechController itself
	service_command_ = controller_nh.advertiseService("mech_posS", &MechController::cmdService, this);

	return true;
}

void MechController::starting(const ros::Time &time) {
	ROS_ERROR_STREAM("MechController was started");
}

void MechController::update(const ros::Time &time, const ros::Duration &period) {
	//float curr_cmd = *(command_.readFromRT()); //why do we put it into a new variable
	//ROS_ERROR_STREAM("curr_cmd : " << curr_cmd);
	joint_1.setCommand(*(command_[0].readFromRT()));
	joint_2.setCommand(*(command_[1].readFromRT()));
}

void MechController::stopping(const ros::Time &time) {
}

bool MechController::cmdService(mech_controller::SetTwoMotors::Request &req, mech_controller::SetTwoMotors::Response &/*response*/) {
	if(isRunning())
		command_.writeFromNonRT(req.values);
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
		return false;
	}
	return true;
}

void MechController::mechPosCallback(const mech_controller::TwoMotor &command) {
	if(isRunning())
		command_.writeFromNonRT(command.values);
	else
		ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS( mech_controller::MechController, controller_interface::ControllerBase)
