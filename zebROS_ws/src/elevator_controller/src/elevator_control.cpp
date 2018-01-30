//this file exists so src folder is uploaded for structure
#include <elevator_controller/linear_control.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


namespace elevator_controller
{
ElevatorController::ElevatorController():
	if_cube_(false),
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
	
	
	std::string lift_name;
	std::string pivot_name;
	std::string intake_name;
	controller_nh.getParam("lift", lift_name);
	controller_nh.getParam("intake", intake_name);
	controller_nh.getParam("pivot", pivot_name);
	
	
	controller_nh.getParam("arm_length", arm_length_);
	
	ros::NodeHandle lnh(controller_nh, lift_name);
	lift_offset_ = 0;
	if (!lnh.getParam("offset", lift_offset_))
                        ROS_ERROR_STREAM("Can not read offset for " << lift_name);
		
	ros::NodeHandle pnh(controller_nh, pivot_name);
	pivot_offset_ = 0;
	if (!pnh.getParam("offset", pivot_offset_))
                        ROS_ERROR_STREAM("Can not read offset for " << pivot_name);

	//Offset for arm should be the angle at arm all the way up, faces flush, - pi / 2
	//Offset for lift should be lift sensor pos when all the way down + height of carriage pivot point 
	//when all the way down


	ROS_INFO_STREAM_NAMED(name_,
                             "Adding pivot with joint name: "   << pivot_name
                             << " and lift with joint name: "   << lift_name
			     << " and intake with joint name: " << intake_name);
	ros::NodeHandle l_nh(controller_nh, pivot_name);
	pivot_joint_.initWithNode(hw, nullptr, l_nh);		
	ros::NodeHandle p_nh(controller_nh, lift_name);
	lift_joint_.initWithNode(hw, nullptr, p_nh);		
	ros::NodeHandle i_nh(controller_nh, intake_name);
	intake_joint_.initWithNode(hw, nullptr, i_nh);		
		
	controller_nh.getParam("max_extension", max_extension_);
	controller_nh.getParam("max_extension", min_extension_);
	
	dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config confP;
        dynamic_reconfigure::Config confL;

        //soft limits need to be enabled in config file

        double_param.name = "softlimit_forward_threshold";
        double_param.value = M_PI/2 + pivot_offset_;
        confP.doubles.push_back(double_param);

        double_param.name = "softlimit_reverse_threshold";
        double_param.value = -M_PI/2 + pivot_offset_;
        confP.doubles.push_back(double_param);


        double_param.name = "softlimit_forward_threshold";
        double_param.value = max_extension_ + lift_offset_;
        confL.doubles.push_back(double_param);

        double_param.name = "softlimit_reverse_threshold";
        double_param.value = min_extension_ + lift_offset_;
        confL.doubles.push_back(double_param);

        srv_req.config = confP;

        ros::service::call("/frcrobot/pivot/updates", srv_req, srv_resp);

        srv_req.config = confL;

        ros::service::call("/frcrobot/lift/updates", srv_req, srv_resp);
	
	//Set soft limits using offsets here

	//unit conversion will work using conversion_factor
	//, not soft limits

	//TODO: something here to get bounding boxes etc.

	
	sub_command_ = controller_nh.subscribe("cmd_pos", 1, &ElevatorController::cmdPosCallback, this);
	
	//TODO: add odom init stuff

	return true;
}
void ElevatorController::update(const ros::Time &time, const ros::Duration &period)
{
	const double delta_t = period.toSec();
        const double inv_delta_t = 1 / delta_t;
	//compOdometry(time, inv_delta_t);
	Commands curr_cmd = *(command_.readFromRT());
	//Use known info to write to hardware etc.
	//Put in intelligent bounds checking 

	





}
void ElevatorController::starting(const ros::Time &time)
{
	//maybe initialize the target to something if not otherwise set?
	//We will need to write this time to some variables for odom eventually
}
void ElevatorController::cmdPosCallback(const elevator_controller::ElevatorControl &command)
{
	if(isRunning())
	{
		command_struct_.lin[0] = command.x;
		command_struct_.lin[1] = command.y;
		command_struct_.up_or_down = command.up_or_down;
		command_struct_.stamp = ros::Time::now();
		command_.writeFromNonRT(command_struct_);
	}	
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}
void ElevatorController::clampCallback(const std_msgs::Bool &command)
{

	if(isRunning())
	{
		clamp_cmd_ = command.data;

	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}
void ElevatorController::intakePowerCallback(const std_msgs::Float64 &command)
{

	if(isRunning())
	{
		intake_power_ = command.data;

	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}
}//Namespace
