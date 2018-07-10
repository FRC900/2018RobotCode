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
	
    controller_nh.param("joint_names", joint_names, std::vector<std::string>());
	joints.resize(joint_names.size());
	//init the joint with the tci, tsi (not used), the node handle, and dynamic reconfigure (t/f)
    for(int i = 0; i<joint_names.size(); i++) {
        ros::NodeHandle l_nh(controller_nh, joint_names[i]);
        if (!joints[i].initWithNode(talon_command_iface, nullptr, l_nh))
        {
            ROS_ERROR("Cannot initialize joint %d!", i);
            return false;
        }
        else
        {
            ROS_INFO("Initialized joint %d!!", i);
        }
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
	mech_controller::TwoMotor final_cmd = *(command_.readFromRT());
    if(final_cmd.values.size() == joints.size()) {
        joints[0].setCommand(final_cmd.values[0]);
        joints[1].setCommand(final_cmd.values[1]);
        ROS_INFO("Hi, I'm alive don't delete me %f, %f", final_cmd.values[0], final_cmd.values[1]);
    }
    else {
        ROS_INFO("Hi, please delete me");
    }
}

void MechController::stopping(const ros::Time &time) {
}

bool MechController::cmdService(mech_controller::SetTwoMotors::Request &req, mech_controller::SetTwoMotors::Response &/*response*/) {
	if(isRunning())
	{
		static mech_controller::TwoMotor temp_cmd;
		temp_cmd.values = req.values;
		command_.writeFromNonRT(temp_cmd);
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
		return false;
	}
	return true;
}

void MechController::mechPosCallback(const mech_controller::TwoMotor &command) {
	if(isRunning())
	{
		static mech_controller::TwoMotor temp_cmd;
		temp_cmd.values = command.values;
		command_.writeFromNonRT(temp_cmd);
	}
	else
		ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS( mech_controller::MechController, controller_interface::ControllerBase)
