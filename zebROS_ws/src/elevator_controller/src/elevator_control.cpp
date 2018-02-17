#include <elevator_controller/linear_control.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


namespace elevator_controller
{
ElevatorController::ElevatorController():
	line_break_intake_index_(-1),
	line_break_clamp_index_(-1),
	line_break_intake_(false),
	line_break_clamp_(false),
	shift_cmd_(false),
	shifted_(false),
	clamp_cmd_(0.0),
	end_game_deploy_cmd_(false),
	end_game_deploy_t1_(false),
	end_game_deploy_t2_(false),
	climb_height_(0.0),
	end_game_deploy_start_(0.0),
	arm_length_(0.0),
	pivot_offset_(0.0),
	lift_offset_(0.0),
	max_extension_(0.0),
	min_extension_(0.0),
	hook_depth_(0.0),
	hook_min_height_(0.0),
	hook_max_height_(0.0),
	intake_down_time_(0.0)
{
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

	ROS_INFO_STREAM("names: " << lift_name << intake_name << pivot_name);

	controller_nh.getParam("arm_length", arm_length_);
	controller_nh.getParam("climb_height", climb_height_);

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
	controller_nh.getParam("min_extension", min_extension_);

	
	controller_nh.getParam("after_shift_max_accel", after_shift_max_accel_);
	controller_nh.getParam("after_shift_max_vel", after_shift_max_vel_);

	controller_nh.getParam("hook_depth", hook_depth_);
	controller_nh.getParam("hook_min_height", hook_min_height_);
	controller_nh.getParam("hook_max_height", hook_max_height_);

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

	ros::service::call("/frcrobot/pivot_joint/parameter_updates", srv_req, srv_resp);

	srv_req.config = confL;

	ros::service::call("/frcrobot/lift_joint/parameter_updates", srv_req, srv_resp);

	//Set soft limits using offsets here

	//unit conversion will work using conversion_factor

	//TODO: something here to get bounding boxes etc for limits near bottom of drive train
	arm_limiting::polygon_type remove_zone_poly_down;
	std::vector<arm_limiting::point_type> point_vector;
	XmlRpc::XmlRpcValue poly_points;
	controller_nh.getParam("polygon_points", poly_points);
	point_vector.resize(poly_points.size()/2);
	//ROS_ERROR("hypothetical errors");
	ROS_INFO_STREAM(poly_points.size());
	for (int i = 0; i < poly_points.size()/2.0; ++i)
	{
		point_vector[i].x(static_cast<double>(poly_points[2*i]));
		point_vector[i].y(static_cast<double>(poly_points[2*i + 1]));
		ROS_INFO_STREAM("point from remove zone: " <<   boost::geometry::wkt(point_vector[i]));
	}
	boost::geometry::assign_points(remove_zone_poly_down, point_vector);
	arm_limiter = std::make_shared<arm_limiting::arm_limits>(min_extension_, max_extension_, 0.0, arm_length_, remove_zone_poly_down, 15);

	sub_command_ = controller_nh.subscribe("cmd_pos", 1, &ElevatorController::cmdPosCallback, this);
	sub_joint_state_ = controller_nh.subscribe("/frcrobot/joint_states", 1, &ElevatorController::lineBreakCallback, this);
	service_command_ = controller_nh.advertiseService("cmd_posS", &ElevatorController::cmdPosService, this);
	service_intake_ = controller_nh.advertiseService("intake", &ElevatorController::intakeService, this);
	service_clamp_ = controller_nh.advertiseService("clamp", &ElevatorController::clampService, this);
	service_shift_ = controller_nh.advertiseService("shift", &ElevatorController::shiftService, this);
	service_end_game_deploy_ = controller_nh.advertiseService("end_game_deploy", &ElevatorController::endGameDeployService, this);



	Clamp      	  = controller_nh.advertise<std_msgs::Float64>("/frcrobot/clamp_controller/command", 1);
	Shift      	  = controller_nh.advertise<std_msgs::Float64>("/frcrobot/shift_controller/command", 1);
	EndGameDeploy     = controller_nh.advertise<std_msgs::Float64>("/frcrobot/end_game_deploy_controller/command", 1);
	CubeState      	  = controller_nh.advertise<std_msgs::Bool>("cube_state", 1);
	IntakeUp      = controller_nh.advertise<std_msgs::Float64>("/frcrobot/intake_up_controller/command", 1);
	IntakeSoftSpring = controller_nh.advertise<std_msgs::Float64>("/frcrobot/intake_spring_soft_controller/command", 1);
	IntakeHardSpring = controller_nh.advertise<std_msgs::Float64>("/frcrobot/intake_spring_hard_controller/command", 1);


	ReturnCmd  = controller_nh.advertise<elevator_controller::ReturnElevatorCmd>("return_cmd_pos", 1);

	Odom  = controller_nh.advertise<elevator_controller::ReturnElevatorCmd>("odom", 1);
	before_shift_max_vel_ = lift_joint_.getMotionCruiseVelocity();
	before_shift_max_accel_ = lift_joint_.getMotionAcceleration();


	return true;
}
void ElevatorController::update(const ros::Time &time, const ros::Duration &period)
{
	//const double delta_t = period.toSec();
	//const double inv_delta_t = 1 / delta_t;
	//compOdometry(time, inv_delta_t);
	if(end_game_deploy_cmd_ && !end_game_deploy_t1_)
	{
		
		command_struct_.lin[0] = .1;
                command_struct_.lin[1] = min_extension_ + cos(asin(.1 / arm_length_))*arm_length_;
                command_struct_.up_or_down = true;
                command_struct_.override_pos_limits = false;
                command_struct_.override_sensor_limits = false;
		
		command_.writeFromNonRT(command_struct_);		

		end_game_deploy_t1_ = true;	
		end_game_deploy_start_ = ros::Time::now().toSec();
	}
	if(end_game_deploy_cmd_ && !end_game_deploy_t2_ && (ros::Time::now().toSec() - end_game_deploy_start_) > .65)
	{
		command_struct_.lin[0] = .1;
                command_struct_.lin[1] = (climb_height_ - min_extension_)*2 + cos(asin(.1 / arm_length_))*arm_length_;
                command_struct_.up_or_down = true;
                command_struct_.override_pos_limits = false;
                command_struct_.override_sensor_limits = false;
		
		command_.writeFromNonRT(command_struct_);		
		end_game_deploy_t2_ = true;	
	}
	if(end_game_deploy_cmd_ && (ros::Time::now().toSec() - end_game_deploy_start_) > .5)
	{	
		EndGameDeploy.publish(1.0);
	}
	else
	{
		EndGameDeploy.publish(0.0);
	}
	if(end_game_deploy_cmd_ && (ros::Time::now().toSec() - end_game_deploy_start_) > 1.0)
	{
		shift_cmd_ = true;
	} 
	if(shift_cmd_)
	{	
		Shift.publish(1.0);
		if(!shifted_)
		{
			shifted_ = true;
			lift_joint_.setMotionAcceleration(after_shift_max_accel_);
			lift_joint_.setMotionCruiseVelocity(after_shift_max_vel_);
			lift_joint_.setPIDFSlot(1.0);
		}
	}
	else
	{
		Shift.publish(-1.0);
		if(shifted_)
		{
			shifted_ = false;
			lift_joint_.setMotionAcceleration(before_shift_max_accel_);
			lift_joint_.setMotionCruiseVelocity(before_shift_max_vel_);
			lift_joint_.setPIDFSlot(0.0);
		}
	}
	Commands curr_cmd = *(command_.readFromRT());
	//Use known info to write to hardware etc.
	//Put in intelligent bounds checking




	intake_joint_.setCommand(intake_struct_.power);


	if(intake_struct_.up_command < 0)
	{
		IntakeUp.publish(-1.0);
		intake_down_time_ = ros::Time::now().toSec();
	}
	else
	{
		if((ros::Time::now().toSec() - intake_down_time_) < .25)
		{
			IntakeUp.publish(1.0);
		}
		else
		{
			IntakeUp.publish(0.0);
		}
	}
	//Delay stuff maybe?

	switch(intake_struct_.spring_command)
	{
		default: IntakeSoftSpring.publish(1.0);
			 IntakeHardSpring.publish(0.0);
		case 1:  IntakeSoftSpring.publish(0.0);
			 IntakeHardSpring.publish(-1.0);
		case 3:  IntakeSoftSpring.publish(0.0);
			 IntakeHardSpring.publish(1.0);
			  
	}	

	Clamp.publish(clamp_cmd_);
	CubeState.publish(line_break_intake_ || line_break_clamp_);


	elevator_controller::ReturnElevatorCmd return_holder;
	elevator_controller::ReturnElevatorCmd odom_holder;

	lift_position = lift_joint_.getPosition() - lift_offset_;
	pivot_angle = pivot_joint_.getPosition() - pivot_offset_;

	bool cur_up_or_down = pivot_angle > 0;

	arm_limiting::point_type cur_pos(cos(pivot_angle)*arm_length_, lift_position +
			sin(pivot_angle)*arm_length_);

	odom_holder.x = cur_pos.x();
	odom_holder.y = cur_pos.y();
	odom_holder.up_or_down = cur_up_or_down;

	Odom.publish(odom_holder);

	if(!curr_cmd.override_pos_limits)
	{


		arm_limiting::point_type cmd_point(curr_cmd.lin[0], curr_cmd.lin[1]);




		bool reassignment_holder;

		arm_limiting::point_type return_cmd;
		bool return_up_or_down;
		arm_limiter->safe_cmd(cmd_point, curr_cmd.up_or_down, reassignment_holder, cur_pos, cur_up_or_down, hook_depth_, hook_min_height_, hook_max_height_, return_cmd, return_up_or_down);

		return_holder.x = return_cmd.x();
		return_holder.y = return_cmd.y();
		return_holder.up_or_down = return_up_or_down;

		//potentially do something if reassignment is needed (Like a ROS_WARN?)

		curr_cmd.lin[0] = cmd_point.x();
		curr_cmd.lin[1] = cmd_point.y();
	}
	else
	{
		return_holder.x = curr_cmd.lin[0];
		return_holder.y = curr_cmd.lin[1];
		return_holder.up_or_down = curr_cmd.up_or_down;

	}
	ReturnCmd.publish(return_holder);

	if(!curr_cmd.override_sensor_limits)
	{
		//TODO: something here which reads time of flight/ultrasonic pos
		//will only go up/down to within 15 cm
		//if target is beyond dist, will bring arm all the way up or down to go around
		//this is relatively low priority
	}
	double pivot_target = acos(curr_cmd.lin[0]/arm_length_) * ((curr_cmd.up_or_down) ? 1 : -1);
	pivot_joint_.setCommand(pivot_target + pivot_offset_);
	lift_joint_.setCommand(curr_cmd.lin[1] - arm_length_ * sin(pivot_target) + lift_offset_);

}
void ElevatorController::starting(const ros::Time &time)
{
	//maybe initialize the target to something if not otherwise set?
}
void ElevatorController::cmdPosCallback(const elevator_controller::ElevatorControl &command)
{
	if(isRunning())
	{
		command_struct_.lin[0] = command.x;
		command_struct_.lin[1] = command.y;
		command_struct_.up_or_down = command.up_or_down;
		command_struct_.override_pos_limits = command.override_pos_limits;
		command_struct_.override_sensor_limits = command.override_sensor_limits;

		command_struct_.stamp = ros::Time::now();
		command_.writeFromNonRT(command_struct_);
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}
void ElevatorController::lineBreakCallback(const sensor_msgs::JointState &msg)
{
	if(isRunning())
	{
		if(line_break_intake_index_ != -1)
		{
			line_break_intake_ = msg.position[line_break_intake_index_] > 0;
		}
		else
		{
			for(int i = 0; i < msg.name.size(); i++)
			{
				if(msg.name[i] == "intake_line_break")
				{
					line_break_intake_index_ = i;
					break;
				}
			}
		}
		if(line_break_clamp_index_ != -1)
		{
			line_break_clamp_ = msg.position[line_break_clamp_index_] > 0;
		}
		else
		{
			for(int i = 0; i < msg.name.size(); i++)
			{
				if(msg.name[i] == "clamp_line_break")
				{
					line_break_clamp_index_ = i;
					break;
				}
			}
		}
	}
}
bool ElevatorController::cmdPosService(elevator_controller::ElevatorControlS::Request &command, elevator_controller::ElevatorControlS::Response &res)
{
	if(isRunning())
	{
		command_struct_.lin[0] = command.x;
		command_struct_.lin[1] = command.y;
		command_struct_.up_or_down = command.up_or_down;
		command_struct_.override_pos_limits = command.override_pos_limits;
		command_struct_.override_sensor_limits = command.override_sensor_limits;

		command_struct_.stamp = ros::Time::now();
		command_.writeFromNonRT(command_struct_);
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}
bool ElevatorController::clampService(elevator_controller::bool_srv::Request &command, elevator_controller::bool_srv::Response &res)
{

	if(isRunning())
	{
		if(command.data)
		{
			clamp_cmd_ = 1.0;
		}
		else
		{
			clamp_cmd_ = -1.0;
		}
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}
bool ElevatorController::shiftService(elevator_controller::bool_srv::Request &command, elevator_controller::bool_srv::Response &res)
{

	if(isRunning())
	{
		shift_cmd_ = command.data;
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}
bool ElevatorController::endGameDeployService(elevator_controller::Blank::Request &command, elevator_controller::Blank::Response &res)
{



	if(isRunning())
	{
		end_game_deploy_cmd_ = true;
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}
bool ElevatorController::intakeService(elevator_controller::Intake::Request &command, elevator_controller::Intake::Response &res)
{

	if(isRunning())
	{
		intake_struct_.power = command.power;

		if(command.up)
		{
			intake_struct_.up_command = -1.0;
		}
		else
		{
			intake_struct_.up_command = 1.0;
		}

		intake_struct_.spring_command = command.spring_state;	
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}
}//Namespace
