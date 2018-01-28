//this file exists so src folder is uploaded for structure
#include "elevator_node/linear_control.h"

namespace elevator_controller
{
ElevatorController::ElevatorController():
	if_cube(false),
	clamp_cmd_(false),
	intake_power_(0.0),
	arm_length_(0.0)
{
}	

void ElevatorController::evaluateCubeState(){
     //TODO : get state of linebreak and publish cube holding state
}

 
bool ElevatorController::init(hardware_interface::TalonCommandInterface *hw,
                			             ros::NodeHandle &root_nh,
             	                        	     ros::NodeHandle &controller_nh)
{
	const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);
	
	
	string lift_name;
	string pivot_name;
	string intake_name;
	controller_nh.getParam("lift", lift_name);
	controller_nh.getParam("intake", intake_name);
	controller_nh.getParam("pivot", pivot_name);
	
	
	controller_nh.getParam("arm_length", arm_length_);
	
	ros::NodeHandle nh(controller_nh, lift_name);
	lift_offset_ = 0;
	if (!nh.getParam("offset", lift_offset_))
                        ROS_ERROR_STREAM("Can not read offset for " << lift_name);
		
	ros::NodeHandle nh(controller_nh, pivot_name);
	pivot_offset_ = 0;
	if (!nh.getParam("offset", pivot_offset_))
                        ROS_ERROR_STREAM("Can not read offset for " << pivot_name);

	ROS_INFO_STREAM_NAMED(name_,
                             "Adding pivot with joint name: "   << pivot_name
                             << " and lift with joint name: "   << lift_name
			     << " and intake with joint name: " << intake_name);
	ros::NodeHandle l_nh(controller_nh, pivot_name);
	pivot_joint_.initWithNode(hw, nullptr, l_nh);		
	ros::NodeHandle l_nh(controller_nh, lift_name);
	lift_joint_.initWithNode(hw, nullptr, l_nh);		
	ros::NodeHandle l_nh(controller_nh, intake_name);
	intake_joint_.initWithNode(hw, nullptr, l_nh);		
	
	
	
	//TODO: something here to get bounding boxes etc.

	

/* ros::init(argc, argv, "elevator_control");
  ros::NodeHandle n;
  RobotStatePub = n.advertise<elevator_teleop_control::RobotState>("RobotState", 1);
  ros::Rate loop_rate(10);
  RobotStatePub.publish(RobotStateMsg);
  ros::spin();*/
}


}
