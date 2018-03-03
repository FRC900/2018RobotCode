#include <elevator_controller/linear_control.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/console.h>



namespace elevator_controller
{
ElevatorController::ElevatorController():
	after_shift_max_accel_(0),
	after_shift_max_vel_(0),
	before_shift_max_accel_(0),
	before_shift_max_vel_(0),
	line_break_intake_(false),
	line_break_clamp_(false),
	shift_cmd_(false),
	shifted_(false),
	clamp_cmd_(0.0),
	climb_height_(0.0),
	end_game_deploy_cmd_(false),
	end_game_deploy_t1_(false),
	end_game_deploy_t2_(false),
	end_game_deploy_start_(0.0),
	max_extension_(0.0),
	min_extension_(0.0),
	intake_down_time_(0.0),
	hook_depth_(0.0),
	hook_min_height_(0.0),
	hook_max_height_(0.0),
	arm_length_(0.0),
	pivot_offset_(0.0),
	lift_offset_(0.0),
	cut_off_line_(0.0)
{
}

bool ElevatorController::getFirstString(XmlRpc::XmlRpcValue value, std::string &str)
{
	if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		if (value.size() == 0)
		{
			ROS_ERROR_STREAM_NAMED(name_,
					"Joint param is an empty list");
			return false;
		}

		if (value[0].getType() != XmlRpc::XmlRpcValue::TypeString)
		{
			ROS_ERROR_STREAM_NAMED(name_,
					"Joint param #0 isn't a string.");
			return false;
		}
		str = static_cast<std::string>(value[0]);
	}
	else if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
	{
		str = static_cast<std::string>(value);
	}
	else
	{
		ROS_ERROR_STREAM_NAMED(name_,
				"Joint param is neither a list of strings nor a string.");
		return false;
	}

	return true;
}

bool ElevatorController::init(hardware_interface::TalonCommandInterface *hw,
                			  ros::NodeHandle &/*root_nh*/,
             	              ros::NodeHandle &controller_nh)
{
	const std::string complete_ns = controller_nh.getNamespace();
	std::size_t id = complete_ns.find_last_of("/");
	name_ = complete_ns.substr(id + 1);

	XmlRpc::XmlRpcValue lift_params;
	if (!controller_nh.getParam("lift", lift_params))
	{
		ROS_ERROR_NAMED(name_, "Can not read lift name(s)");
		return false;
	}
	XmlRpc::XmlRpcValue intake_params;
	if (!controller_nh.getParam("intake", intake_params))
	{
		ROS_ERROR_NAMED(name_, "Can not read intake name(s)");
		return false;
	}
	XmlRpc::XmlRpcValue pivot_params;
	if (!controller_nh.getParam("pivot", pivot_params))
	{
		ROS_ERROR_NAMED(name_, "Can not read pivot name(s)");
		return false;
	}

	if (!controller_nh.getParam("arm_length", arm_length_))
	{
		ROS_ERROR_NAMED(name_, "Can not read arm_length");
		return false;
	}
	if (!controller_nh.getParam("climb_height", climb_height_))
	{
		ROS_ERROR_NAMED(name_, "Can not read climb_height");
		return false;
	}
	if (!controller_nh.getParam("cut_off_line", cut_off_line_))
	{
		ROS_ERROR_NAMED(name_, "Can not read cut_off_line");
		return false;
	}

	std::string node_name;
	//Offset for lift should be lift sensor pos when all the way down + height of carriage pivot point
	//when all the way down
	lift_offset_ = 0;
	if (getFirstString(lift_params, node_name) &&
	   !ros::NodeHandle(controller_nh, node_name).getParam("offset", lift_offset_))
	{
		ROS_ERROR_STREAM("Can not read offset for lift " << node_name);
		return false;
	}

	//Offset for arm should be the angle at arm all the way up, faces flush, - pi / 2
	pivot_offset_ = 0;
	if (getFirstString(pivot_params, node_name) &&
	   !ros::NodeHandle(controller_nh, node_name).getParam("offset", pivot_offset_))
	{
		ROS_ERROR_STREAM("Can not read offset for pivot " << node_name);
		return false;
	}
	
	if (!pivot_joint_.initWithNode(hw, nullptr, controller_nh, pivot_params))
	{
		ROS_ERROR("Can not initialize pivot joint(s)");
		return false;
	}

	if (!lift_joint_.initWithNode(hw, nullptr, controller_nh, lift_params))
	{
		ROS_ERROR("Can not initialize lift joint(s)");
		return false;
	}
	if (!intake_joint_.initWithNode(hw, nullptr, controller_nh, intake_params))
	{
		ROS_ERROR("Can not initialize intake joint(s)");
		return false;
	}
	
	if (!controller_nh.getParam("max_extension", max_extension_))
	{
		ROS_ERROR_NAMED(name_, "Can not read max_extension");
		return false;
	}
	if (!controller_nh.getParam("min_extension", min_extension_))
	{
		ROS_ERROR_NAMED(name_, "Can not read min_extension");
		return false;
	}

	if (!controller_nh.getParam("after_shift_max_accel", after_shift_max_accel_))
	{
		ROS_ERROR_NAMED(name_, "Can not read shift_max_accel");
		return false;
	}
	if (!controller_nh.getParam("after_shift_max_vel", after_shift_max_vel_))
	{
		ROS_ERROR_NAMED(name_, "Can not read shift_min_accel");
		return false;
	}

	if (!controller_nh.getParam("hook_depth", hook_depth_))
	{
		ROS_ERROR_NAMED(name_, "Can not read hook_depth");
		return false;
	}
	if (!controller_nh.getParam("hook_min_height", hook_min_height_))
	{
		ROS_ERROR_NAMED(name_, "Can not read hook_min_height");
		return false;
	}
	if (!controller_nh.getParam("hook_max_height", hook_max_height_))
	{
		ROS_ERROR_NAMED(name_, "Can not read hook_max_height");
		return false;
	}
	
	//Set soft limits using offsets here
	lift_joint_.setForwardSoftLimitThreshold(max_extension_ + lift_offset_);
	lift_joint_.setReverseSoftLimitThreshold(min_extension_ + lift_offset_);
	lift_joint_.setForwardSoftLimitEnable(true);
	lift_joint_.setReverseSoftLimitEnable(true);

	pivot_joint_.setForwardSoftLimitThreshold(M_PI/2 -.05 + pivot_offset_);
	pivot_joint_.setReverseSoftLimitThreshold(-M_PI/2 + .05 + pivot_offset_);
	
	//TODO: something is broke with these soft limits

	pivot_joint_.setForwardSoftLimitEnable(true);
	pivot_joint_.setReverseSoftLimitEnable(true);
	
	//unit conversion will work using conversion_factor

	//TODO: something here to get bounding boxes etc for limits near bottom of drive train
	arm_limiting::polygon_type remove_zone_poly_down;
	std::vector<arm_limiting::point_type> point_vector;
	XmlRpc::XmlRpcValue poly_points;
	if (!controller_nh.getParam("polygon_points", poly_points))
	{
		ROS_ERROR_NAMED(name_, "Can not read polygon_points");
		return false;
	}
	point_vector.resize(poly_points.size()/2);
	//ROS_ERROR_NAMED(name_, "hypothetical errors");
	ROS_INFO_STREAM("Poly_points " << std::endl << poly_points.size());
	for (int i = 0; i < poly_points.size()/2; ++i)
	{
		point_vector[i].x(static_cast<double>(poly_points[2*i]));
		point_vector[i].y(static_cast<double>(poly_points[2*i + 1]));
		ROS_INFO_STREAM("point from remove zone: " << boost::geometry::wkt(point_vector[i]));
	}
	boost::geometry::assign_points(remove_zone_poly_down, point_vector);
	arm_limiter_ = std::make_shared<arm_limiting::arm_limits>(min_extension_, max_extension_, 0.0, arm_length_, remove_zone_poly_down, 15);

	sub_command_ = controller_nh.subscribe("cmd_pos", 1, &ElevatorController::cmdPosCallback, this);
	sub_stop_arm_ = controller_nh.subscribe("stop_arm", 1, &ElevatorController::stopCallback, this);
	sub_joint_state_ = controller_nh.subscribe("/frcrobot/joint_states", 1, &ElevatorController::lineBreakCallback, this);
	service_command_ = controller_nh.advertiseService("cmd_posS", &ElevatorController::cmdPosService, this);
	service_intake_ = controller_nh.advertiseService("intake", &ElevatorController::intakeService, this);
	service_clamp_ = controller_nh.advertiseService("clamp", &ElevatorController::clampService, this);
	service_shift_ = controller_nh.advertiseService("shift", &ElevatorController::shiftService, this);
	service_end_game_deploy_ = controller_nh.advertiseService("end_game_deploy", &ElevatorController::endGameDeployService, this);

	Clamp_     	      = controller_nh.advertise<std_msgs::Float64>("/frcrobot/clamp_controller/command", 1);
	Shift_     	      = controller_nh.advertise<std_msgs::Float64>("/frcrobot/shift_controller/command", 1);
	EndGameDeploy_    = controller_nh.advertise<std_msgs::Float64>("/frcrobot/end_game_deploy_controller/command", 1);
	CubeState_        = controller_nh.advertise<std_msgs::Bool>("cube_state", 1);
	IntakeUp_         = controller_nh.advertise<std_msgs::Float64>("/frcrobot/intake_up_controller/command", 1);
	IntakeSoftSpring_ = controller_nh.advertise<std_msgs::Float64>("/frcrobot/intake_spring_soft_controller/command", 1);
	IntakeHardSpring_ = controller_nh.advertise<std_msgs::Float64>("/frcrobot/intake_spring_hard_controller/command", 1);
	ReturnCmd_        = controller_nh.advertise<elevator_controller::ReturnElevatorCmd>("return_cmd_pos", 1);

	Odom_             = controller_nh.advertise<elevator_controller::ReturnElevatorCmd>("odom", 1);
	before_shift_max_vel_ = lift_joint_.getMotionCruiseVelocity();
	before_shift_max_accel_ = lift_joint_.getMotionAcceleration();

	return true;
}

void ElevatorController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
	//const double delta_t = period.toSec();
	//const double inv_delta_t = 1 / delta_t;
	//compOdometry(time, inv_delta_t);
	if(end_game_deploy_cmd_ && !end_game_deploy_t1_)
	{
		
		ROS_INFO("part 1");
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
		ROS_INFO("part 2");
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
		ROS_INFO("dropping");
		std_msgs::Float64 msg;
		msg.data = 1.0;
		EndGameDeploy_.publish(msg);
	}
	else
	{
		std_msgs::Float64 msg;
		msg.data = 0.0;
		EndGameDeploy_.publish(msg);
	}
	if(end_game_deploy_cmd_ && (ros::Time::now().toSec() - end_game_deploy_start_) > 1.0)
	{
		shift_cmd_ = true;
		ROS_INFO("shifting");
	} 
	if(shift_cmd_)
	{	
		std_msgs::Float64 msg;
		msg.data = 1.0;
		Shift_.publish(msg);
		if(!shifted_)
		{
			shifted_ = true;
			lift_joint_.setMotionAcceleration(after_shift_max_accel_);
			lift_joint_.setMotionCruiseVelocity(after_shift_max_vel_);
			lift_joint_.setPIDFSlot(1);
		}
	}
	else
	{
		std_msgs::Float64 msg;
		msg.data = -1.0;
		Shift_.publish(msg);
		if(shifted_)
		{
			shifted_ = false;
			lift_joint_.setMotionAcceleration(before_shift_max_accel_);
			lift_joint_.setMotionCruiseVelocity(before_shift_max_vel_);
			lift_joint_.setPIDFSlot(0);
		}
	}
	Commands curr_cmd = *(command_.readFromRT());
	cur_intake_cmd_ = *(intake_command_.readFromRT());
	//Use known info to write to hardware etc.
	//Put in intelligent bounds checking

	intake_joint_.setCommand(cur_intake_cmd_.power);

	if(cur_intake_cmd_.up_command < 0)
	{
		std_msgs::Float64 msg;
		msg.data = -1.0;
		IntakeUp_.publish(msg);
		intake_down_time_ = ros::Time::now().toSec();
	}
	else
	{
		//if((ros::Time::now().toSec() - intake_down_time_) < 1.5) //1.5 is super arbitary
		//{
			std_msgs::Float64 msg;
			msg.data = 1.0;
			IntakeUp_.publish(msg);
		//}
		/*else
		{
			std_msgs::Float64 msg;
			msg.data = 0;
			IntakeUp_.publish(msg);
		}*/
	}
	//Delay stuff maybe?
	


	std_msgs::Float64 intake_soft_msg;
	std_msgs::Float64 intake_hard_msg;
	switch(cur_intake_cmd_.spring_command)
	{
		default:
			intake_soft_msg.data = 1.0;
			intake_hard_msg.data = 1.0;
			break;
		case 1:
			intake_soft_msg.data = 1.0;
			intake_hard_msg.data = -1.0;
			break;
		case 3:
			intake_soft_msg.data = 0.0;
			intake_hard_msg.data = 1.0;
			break;
	}	
	IntakeSoftSpring_.publish(intake_soft_msg);
	IntakeHardSpring_.publish(intake_hard_msg);

	std_msgs::Float64 clamp_msg;
	clamp_msg.data = clamp_cmd_;
	Clamp_.publish(std_msgs::Float64(clamp_msg));

	std_msgs::Bool cube_msg;
	cube_msg.data = line_break_intake_ || line_break_clamp_;
	CubeState_.publish(cube_msg);

	elevator_controller::ReturnElevatorCmd return_holder;
	elevator_controller::ReturnElevatorCmd odom_holder;

	const double lift_position =  /*last_tar_l - lift_offset_;*/ lift_joint_.getPosition()  - lift_offset_;
	double pivot_angle   =  /*last_tar_p - pivot_offset_;*/ pivot_joint_.getPosition() - pivot_offset_;

		

	//BELOW IS TOTAL HACK
	//if(fabs(pivot_angle) < .0025)
	//{
	//	pivot_angle +=.005;
	//
	//}
	//ROS_INFO_STREAM("lift_pos: " << lift_position);

	bool cur_up_or_down = pivot_angle > 0;


	arm_limiting::point_type cur_pos(cos(pivot_angle)*arm_length_, lift_position +
			sin(pivot_angle)*arm_length_);

	odom_holder.x = cur_pos.x();
	odom_holder.y = cur_pos.y();
	odom_holder.up_or_down = cur_up_or_down;

	Odom_.publish(odom_holder);
	
	if(stop_arm_)
	{	
		pivot_joint_.setPeakOutputForward(0);
		pivot_joint_.setPeakOutputReverse(0);

		lift_joint_.setPeakOutputForward(0);
		lift_joint_.setPeakOutputReverse(0);
	}


	if(!curr_cmd.override_pos_limits)
	{

		arm_limiting::point_type cmd_point(curr_cmd.lin[0], curr_cmd.lin[1]);
		if(cur_pos.y() < cut_off_line_)
		{
			cmd_point.x(cur_pos.x()); //Don't smack stuff	
		}

		bool reassignment_holder;

		arm_limiting::point_type return_cmd;
		bool return_up_or_down;
		arm_limiter_->safe_cmd(cmd_point, curr_cmd.up_or_down, reassignment_holder, cur_pos, cur_up_or_down, hook_depth_, hook_min_height_, hook_max_height_, return_cmd, return_up_or_down);

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
	ReturnCmd_.publish(return_holder);

	if(!curr_cmd.override_sensor_limits)
	{
		//TODO: something here which reads time of flight/ultrasonic pos
		//will only go up/down to within 15 cm
		//if target is beyond dist, will bring arm all the way up or down to go around
		//this is relatively low priority
	}
	//ROS_INFO_STREAM("cmd: " << curr_cmd.lin << " up/down: " << curr_cmd.up_or_down);
	const double pivot_target = acos(curr_cmd.lin[0]/arm_length_) * ((curr_cmd.up_or_down) ? 1 : -1);
	
	//ROS_INFO_STREAM("up_or_down: " << curr_cmd.up_or_down << "lin pos target" << curr_cmd.lin << " lift pos tar: " << curr_cmd.lin[1] - arm_length_ * sin(pivot_target));	

	


	pivot_joint_.setCommand(pivot_target + pivot_offset_);
	lift_joint_.setCommand(curr_cmd.lin[1] - arm_length_ * sin(pivot_target) + lift_offset_);
	last_tar_l = curr_cmd.lin[1] - arm_length_ * sin(pivot_target) + lift_offset_;
	last_tar_p = (pivot_target + pivot_offset_);
}
void ElevatorController::starting(const ros::Time &/*time*/)
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
void ElevatorController::stopCallback(const std_msgs::Bool &command)
{
	if(isRunning())
	{
		stop_arm_ = command.data;
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
		static size_t line_break_intake_index = -1;
		if(line_break_intake_index == -1)
		{
			for(size_t i = 0; i < msg.name.size(); i++)
			{
				if(msg.name[i] == "intake_line_break")
				{
					line_break_intake_index = i;
					break;
				}
			}
		}
		if(line_break_intake_index == -1)
			ROS_ERROR_NAMED(name_, "Could not read index for intake_line_break");
		else
			line_break_intake_ = msg.position[line_break_intake_index] > 0;

		static size_t line_break_clamp_index = -1;
		if(line_break_clamp_index == -1)
		{
			for(size_t i = 0; i < msg.name.size(); i++)
			{
				if(msg.name[i] == "clamp_line_break")
				{
					line_break_clamp_index = i;
					break;
				}
			}
		}
		if(line_break_clamp_index == -1)
			ROS_ERROR_NAMED(name_, "Could not read index for clamp_line_break");
		else
			line_break_clamp_ = msg.position[line_break_clamp_index] > 0;
	}
}

bool ElevatorController::cmdPosService(elevator_controller::ElevatorControlS::Request &command, elevator_controller::ElevatorControlS::Response &/*res*/)
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

bool ElevatorController::clampService(elevator_controller::bool_srv::Request &command, elevator_controller::bool_srv::Response &/*res*/)
{
	if(isRunning())
	{
		if(command.data)
		{
			clamp_cmd_ = -1.0;
		}
		else
		{
			clamp_cmd_ = 1.0;
		}
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}

bool ElevatorController::shiftService(elevator_controller::bool_srv::Request &command, elevator_controller::bool_srv::Response &/*res*/)
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

bool ElevatorController::endGameDeployService(elevator_controller::Blank::Request &/*command*/, elevator_controller::Blank::Response &/*res*/)
{
	if(isRunning())
	{
		ROS_WARN("called deploy");
		end_game_deploy_cmd_ = true;
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}

bool ElevatorController::intakeService(elevator_controller::Intake::Request &command, elevator_controller::Intake::Response &/*res*/)
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
		intake_command_.writeFromNonRT(intake_struct_);
		intake_down_time_ = ros::Time::now().toSec();
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}
}//Namespace
