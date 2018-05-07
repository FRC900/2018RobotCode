#include <atomic>

#include <behaviors/autoInterpreterClient.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/RobotAction.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros_control_boilerplate/AutoModeStatus.h>
#include <std_msgs/Float64.h>
#include <robot_visualizer/ProfileFollower.h>


//enum Action {deploy_intake, undeploy_intake, intake_cube, intake_no_arm, exchange_cube, default_config, intake_config, switch_config, low_scale_config, mid_scale_config, high_scale_config, over_back_config, release_clamp};

std::atomic<bool> exit_auto; // robot is disabled or teleop started during auto_loop
//std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::RobotAction>> ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> ac_intake;

ros::ServiceClient point_gen;
ros::ServiceClient swerve_control;

static ros::ServiceClient IntakeService;
static ros::ServiceClient ElevatorService;
static ros::ServiceClient ClampService;
static ros::ServiceClient BrakeService;
static ros::ServiceClient VisualizeService;



static ros::Publisher VelPub;
static ros::Publisher auto_state_0;
static ros::Publisher auto_state_1;
static ros::Publisher auto_state_2;
static ros::Publisher auto_state_3;

double high_scale_config_x;
double high_scale_config_y;
double mid_scale_config_x;
double mid_scale_config_y;
double low_scale_config_x;
double low_scale_config_y;
double switch_config_x;
double switch_config_y;
double exchange_config_x;
double exchange_config_y;
double intake_ready_to_drop_x;
double intake_ready_to_drop_y;
double over_back_x;
double over_back_y;
bool intake_ready_to_drop_up_or_down;
double default_x;
double default_y;
double run_out_start;
static bool default_up_or_down;
std::vector<double> auto_mode_status_vect = {1, 1, 1, 1};

std::vector<std::vector<trajectory_msgs::JointTrajectory>> joint_trajectories;


struct MatchData {
    MatchData():
        isEnabled_(false),
        isAutonomous_(false),
        alliance_data_("")
    {
    }
    
    MatchData(bool isEnabled, bool isAutonomous, const std::string &alliance_data):
        isEnabled_(isEnabled),
        isAutonomous_(isAutonomous),
        alliance_data_(alliance_data)
    {
    }
    bool isEnabled_;
    bool isAutonomous_;
    std::string alliance_data_;
};


struct AutoMode {
    AutoMode():
        modes_ {0, 0, 0, 0},
        delays_ {0, 0, 0, 0},
        start_pos_(0)
    {
    }
    
    AutoMode(const std::vector<int> &modes, const std::vector<double> &delays, int start_pos):
        modes_(modes),
        delays_(delays),
        start_pos_(start_pos)
    {
    }
    
    std::vector<int> modes_;
    std::vector<double> delays_;
    int start_pos_;
};

realtime_tools::RealtimeBuffer<MatchData> matchData;
realtime_tools::RealtimeBuffer<int> queue_slot;
realtime_tools::RealtimeBuffer<AutoMode> autoMode;

bool defaultConfig(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = default_x;
    srv.request.y = default_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("def conf");
	if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in defaultConfig");
		return false;
	}
	return true;
}
bool startMatchConfig()
{
	elevator_controller::ElevatorControlS srv;
    srv.request.x = .41;
    srv.request.y = .5;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = true;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("start conf");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in defaultConfig");
		return false;
	}
	return true;


}
bool intakeConfig(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = intake_ready_to_drop_x;
    srv.request.y = intake_ready_to_drop_y;
    srv.request.up_or_down = intake_ready_to_drop_up_or_down;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("intake conf");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in intakeConfig");
		return false;
	}
	return true;
}

bool overBack(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = over_back_x;
    srv.request.y = over_back_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("over back conf");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in intakeConfig");
		return false;
	}
	return true;
}

bool customConfig(const ActionSetpoint &action_setpoint) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = over_back_x;
    srv.request.y = over_back_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("custom conf");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in intakeConfig");
		return false;
	}
	return true;
}

bool intakeCube(const ActionSetpoint &action_setpoint) {
	behaviors::RobotGoal goal;

    ROS_WARN("intaking cube");
    goal.IntakeCube = true;
    goal.MoveToIntakeConfig = false;
    goal.x = action_setpoint.x;
    goal.y = action_setpoint.y;
    goal.up_or_down = action_setpoint.up_or_down;
    goal.override_pos_limits = false; //TODO config these
    goal.dist_tolerance = .1; //TODO config these
    goal.x_tolerance = .1; //TODO config these
    goal.y_tolerance = .1; //TODO config these
    goal.time_out = 15; //TODO config these
	goal.wait_to_proceed = false;

    ac->sendGoal(goal);
    
    ROS_INFO("intake");
}
bool intakeNoArm(void) {
	behaviors::IntakeGoal goal;

    ROS_WARN("intaking cube");
    goal.IntakeCube = true;
    goal.time_out = 10; //TODO config this
    goal.wait_to_proceed = false; 

    ac_intake->sendGoal(goal);
    ROS_INFO("intake no arm");
    
}
bool switchConfig(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = switch_config_x;
    srv.request.y = switch_config_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.put_cube_in_intake = false;
    srv.request.override_sensor_limits = false;
    ROS_INFO("switch");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in switchConfig");
		return false;
	}
	return true;
}
bool highScale(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = high_scale_config_x;
    srv.request.y = high_scale_config_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("high scale");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in highScale");
		return false;
	}
	return true;
}
bool midScale(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = mid_scale_config_x;
    srv.request.y = mid_scale_config_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("mid scale");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in midScale");
		return false;
	}
	return true;
}
bool lowScale(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = low_scale_config_x;
    srv.request.y = low_scale_config_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
    ROS_INFO("low scale");
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in lowScale");
		return false;
	}
	return true;
}
bool intakeStop(void) {
	elevator_controller::Intake srv;
    srv.request.spring_state = 2; //soft_in
    srv.request.up=true;
    srv.request.power=0;
    srv.request.other_power=0;
    srv.request.just_override_power = false;
    ROS_INFO("intake stop");
    if (!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in intakeStop");
		return false;
	}
	return true;
}
bool intakeOpenDown(void) {
	elevator_controller::Intake srv;
    srv.request.spring_state = 1; //hard_out
    srv.request.power=0;
    srv.request.other_power=0;
    srv.request.up = false;
    srv.request.just_override_power = false;
    ROS_INFO("intake open down");
    if (!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in intakeOpenDown");
		return false;
	}
	return true;
}
bool releaseIntake(void) {
	elevator_controller::Intake srv;
    srv.request.spring_state = 1; //hard out
    srv.request.power = 0;
    srv.request.other_power=0;
    srv.request.just_override_power = false;
    ROS_INFO("release intake");
    if (!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in releaseIntake");
		return false;
	}
	return true;
}
bool releaseClamp(void) {
	std_srvs::SetBool srv;
	srv.request.data = false;
    ROS_INFO("release clamp");
	if (!ClampService.call(srv))
	{
		ROS_ERROR("Service call failed : ClampService in releaseClamp");
		return false;
	}
	return true;
}
bool clamp(void) {
	std_srvs::SetBool srv;
    srv.request.data = true;
    ROS_INFO("clamp");
    if (!ClampService.call(srv))
	{
		ROS_ERROR("Service call failed : ClampService in clamp");
		return false;
	}
	return true;
}

bool intakeOut(void) {
	elevator_controller::Intake srv;
	srv.request.power = -1;
    srv.request.other_power=-1;
	srv.request.spring_state = 2; //soft-in
    srv.request.just_override_power = false;
    ROS_INFO("intake_out");
	if(!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in intakeOut");
	}
    run_out_start = ros::Time::now().toSec();
	return true;
}
bool deployIntake(void) {
	elevator_controller::Intake srv;
	srv.request.power = 0;
    srv.request.other_power=0;
	srv.request.spring_state = 2; //soft-in
    srv.request.up = 0; //down
    srv.request.just_override_power = false;
    ROS_INFO("intake_deplot");
	if(!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in deployIntake");
	}
    run_out_start = ros::Time::now().toSec();
	return true;

}
bool undeployIntake(void) {
	elevator_controller::Intake srv;
	srv.request.power = 0;
    srv.request.other_power=0;
	srv.request.spring_state = 2; //soft-in
    srv.request.up = true;
    srv.request.just_override_power = false;
    ROS_INFO("intake up");
	if(!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in undeployIntake");
	}
    run_out_start = ros::Time::now().toSec();
	return true;

}

bool parkingConfig(void)
{
	std_srvs::Empty empty;
	ROS_INFO("Braking");
	if (!BrakeService.call(empty))
	{
		ROS_ERROR("Service call failed : BrakeService in parkingConfig");
		return false;
	}
	return true;
}

Modes load_all_trajectories(int max_mode_num, int max_mode_cmd_vel, int max_start_pos_num, int max_wait_for_action_num, ros::NodeHandle &auto_data, ros::NodeHandle &cmd_vel_data)
{
	XmlRpc::XmlRpcValue mode_xml;
	XmlRpc::XmlRpcValue actions_xml;

	XmlRpc::XmlRpcValue cmd_vel_xml;

	ModeList profiled_modes;
    CmdVelList cmd_vel_modes;

	profiled_modes.resize(max_mode_num + 1);
	for(int mode = 0; mode <= max_mode_num; mode++)
	{
		profiled_modes[mode].resize(4);
		for(int layout = 0; layout <= 3; layout++)
		{
			profiled_modes[mode][layout].resize(max_start_pos_num + 1);
			for(int start_pos = 0; start_pos <= max_start_pos_num; start_pos++)
			{
				profiled_modes[mode][layout][start_pos].srv_msgs.resize(max_wait_for_action_num + 1);
				std::string identifier("mode_"+std::to_string(mode)+"_layout_"+
				std::to_string(layout)+"_start_"+std::to_string(start_pos));
				
				for(int wait_for_action = 0; wait_for_action <= max_wait_for_action_num; wait_for_action++)
				{
					std::string identifier_with_wait = identifier+ +"_wait_for_action_"+std::to_string(wait_for_action);
					if(auto_data.getParam(identifier_with_wait, mode_xml))
					{
						ROS_INFO_STREAM("Auto mode with identifier: " << identifier << " found");
						//XmlRpc::XmlRpcValue &coefs_xml = mode_xml["coefs"];
						//XmlRpc::XmlRpcValue &times_xml = mode_xml["times"];
						const int num_splines = mode_xml.size();
						//const int num_times = times_xml.size();
						for(int num = 0; num<num_splines; num++) {
							XmlRpc::XmlRpcValue &spline = mode_xml[num];
							XmlRpc::XmlRpcValue &x = spline["x"];
							XmlRpc::XmlRpcValue &y = spline["y"];
							XmlRpc::XmlRpcValue &orient = spline["orient"];
							//XmlRpc::XmlRpcValue &time = spline["time"];

							swerve_point_generator::Coefs x_coefs;
							swerve_point_generator::Coefs y_coefs;
							swerve_point_generator::Coefs orient_coefs;
							for(int i = 0; i<x.size(); i++) {
								const double x_coef = x[i];
								const double y_coef = y[i];
								const double orient_coef = orient[i];

								////ROS_WARN("%f", orient_coef);
								x_coefs.spline.push_back(x_coef);
								y_coefs.spline.push_back(y_coef);
								orient_coefs.spline.push_back(orient_coef);
							}
							//const double t = time;
							//profiled_modes[mode][layout][start_pos].times.push_back(t);
							profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.x_coefs.push_back(x_coefs);
							profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.y_coefs.push_back(y_coefs);
							profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.orient_coefs.push_back(orient_coefs);
							profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.end_points.push_back(num+1);
						}
						profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.initial_v = 0; 
						profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.final_v = 0; 
						profiled_modes[mode][layout][start_pos].exists = true; 
						profiled_modes[mode][layout][start_pos].num_srv_msgs +=1; 

						ROS_WARN_STREAM("mode: " << mode << " layout: " << layout << " start_pos: " << start_pos);

						XmlRpc::XmlRpcValue group_xml;
						if(auto_data.getParam(identifier_with_wait + "_spline_group", group_xml))
						{
							//ROS_INFO_STREAM("Custom grouping for identifier: " << identifier << " found");
							for(int i = 0; i < group_xml.size(); i++)
							{	
								profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.spline_groups.push_back(group_xml[i]);
							}
							XmlRpc::XmlRpcValue wait_xml;
							if(auto_data.getParam(identifier_with_wait + "_waits", wait_xml))
							{
								//ROS_INFO_STREAM("Custom waits for identifier: " << identifier << " found");
								for(int i = 0; i < group_xml.size(); i++)
								{
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.wait_before_group.push_back(wait_xml[i]);
								}
							}
							else
							{
								for(int i = 0; i < group_xml.size(); i++)
								{
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.wait_before_group.push_back(0.25);
								}
							}
							XmlRpc::XmlRpcValue shift_xml;
							if(auto_data.getParam(identifier_with_wait + "_t_shifts", shift_xml))
							{
								ROS_INFO_STREAM("Custom shifts for identifier: " << identifier_with_wait << " found");
								for(int i = 0; i < group_xml.size(); i++)
								{
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.t_shift.push_back(shift_xml[i]);
								}
							}
							else
							{
								for(int i = 0; i < group_xml.size(); i++)
								{
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.t_shift.push_back(0);
								}
							}
							XmlRpc::XmlRpcValue flip_xml;
							if(auto_data.getParam(identifier_with_wait + "_flips", flip_xml))
							{
								ROS_INFO_STREAM("Custom flips for identifier: " << identifier_with_wait << " found");
								for(int i = 0; i < group_xml.size(); i++)
								{
									bool temp = flip_xml[i];
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.flip.push_back(temp);
								}
							}
							else
							{
								for(int i = 0; i < group_xml.size(); i++)
								{
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.flip.push_back(false);
								}
							}
							XmlRpc::XmlRpcValue x_inverts_xml;
							if(auto_data.getParam(identifier_with_wait + "_x_inverts", x_inverts_xml))
							{
								ROS_INFO_STREAM("Custom x inverts for identifier: " << identifier_with_wait << " found");
								for(int i = 0; i < group_xml.size(); i++)
								{
									bool temp = x_inverts_xml[i];
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.x_invert.push_back(temp);
								}
							}
							else
							{
								for(int i = 0; i < group_xml.size(); i++)
								{
									profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.x_invert.push_back(false);
								}
							}
						}
						else
						{
								profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.flip.push_back(false);
								profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.x_invert.push_back(false);
								profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.spline_groups.push_back(num_splines);
								profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.wait_before_group.push_back(0.25);
								profiled_modes[mode][layout][start_pos].srv_msgs[wait_for_action].request.t_shift.push_back(0);
						}

						
					

				}
				

			}
			
			if(auto_data.getParam(identifier+ "_actions", actions_xml))
			{
				ROS_INFO_STREAM("Auto mode actions with identifier: " << identifier+"_actions" << " found");
				const int num_actions = actions_xml.size();
				profiled_modes[mode][layout][start_pos].actions.resize(num_actions);
				for(int i = 0; i < max_wait_for_action_num; i++)
				{
					profiled_modes[mode][layout][start_pos].wait_ids.push_back(-1);
				}

				for(int num = 0; num<num_actions; num++) {
					XmlRpc::XmlRpcValue &action = actions_xml[num];
					XmlRpc::XmlRpcValue &time = action["time"];
					
					XmlRpc::XmlRpcValue &profile_wait_xml = action["profile_wait"];
					XmlRpc::XmlRpcValue &action_waiter_xml = action["wait_for_action"];
					XmlRpc::XmlRpcValue &action_waiter_duration_xml = action["wait_for_action_delay"];
					XmlRpc::XmlRpcValue &profile_waiter_xml = action["wait_for_profile"];
					//XmlRpc::XmlRpcValue &profile_waiter_duration_xml = action["wait_for_profile_delay"];
					XmlRpc::XmlRpcValue &x_xml = action["x"];
					XmlRpc::XmlRpcValue &y_xml = action["y"];
					XmlRpc::XmlRpcValue &up_or_down_xml = action["up_or_down"];
					const double profile_wait_double = profile_wait_xml;
					int profile_wait = (int)profile_wait_double;

					
					const double action_waiter_double = action_waiter_xml;
					int action_waiter = (int)action_waiter_double;

				
					
					const double action_waiter_duration_double = action_waiter_duration_xml;
					int action_waiter_duration = (int)action_waiter_duration_double;
	

					const double profile_waiter_double = profile_waiter_xml;
					int profile_waiter = (int)profile_waiter_double;
					
					//TODO: add dur

					profiled_modes[mode][layout][start_pos].actions[num].action_setpoint.x = x_xml;
					profiled_modes[mode][layout][start_pos].actions[num].action_setpoint.y = y_xml;
					profiled_modes[mode][layout][start_pos].actions[num].action_setpoint.up_or_down = up_or_down_xml;
					
					profiled_modes[mode][layout][start_pos].actions[num].wait_action_id = action_waiter;
					profiled_modes[mode][layout][start_pos].actions[num].wait_action_time = action_waiter_duration_xml;

				
					profiled_modes[mode][layout][start_pos].actions[num].wait_profile_id = profile_waiter;
					//profiled_modes[mode][layout][start_pos].actions[num].wait_profile_time = profile_waiter_duration_xml;

					if(profile_wait >= 0)
					{				
						profiled_modes[mode][layout][start_pos].wait_ids[profile_wait] =  num;
					}
					//TODO: params
					XmlRpc::XmlRpcValue &action_name = action["actions"];
						//profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(action_now);
						if(action_name == "deploy_intake") {
							profiled_modes[mode][layout][start_pos].actions[num].action = deploy_intake;
						}
						else if(action_name == "undeploy_intake") {
							profiled_modes[mode][layout][start_pos].actions[num].action = undeploy_intake;
						}
						else if(action_name == "intake_cube") {
							profiled_modes[mode][layout][start_pos].actions[num].action = intake_cube;
						}
						else if(action_name == "exchange_cube") {
							profiled_modes[mode][layout][start_pos].actions[num].action = exchange_cube;
						}
						else if(action_name == "default_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = default_config;
						}
						else if(action_name == "intake_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = intake_config;
						}
						else if(action_name == "exchange_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = exchange_config;
						}
						else if(action_name == "switch_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = switch_config;
						}
						else if(action_name == "low_scale_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = low_scale_config;
						}
						else if(action_name == "mid_scale_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = mid_scale_config;
						}
						else if(action_name == "high_scale_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = high_scale_config;
						}
						else if(action_name == "over_back_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = over_back_config;
						}
						else if(action_name == "custom_config") {
							profiled_modes[mode][layout][start_pos].actions[num].action = custom_config;
						}
						else if(action_name == "release_clamp") {
							profiled_modes[mode][layout][start_pos].actions[num].action = release_clamp;
						}
						else if(action_name == "intake_open_down") {
							profiled_modes[mode][layout][start_pos].actions[num].action = intake_open_down;
						}
						else if(action_name == "intake_no_arm") {
							profiled_modes[mode][layout][start_pos].actions[num].action = intake_no_arm;
						}
						else
						{
							ROS_ERROR("action not found");
						}
						
					//Exception handling?
					const double time_now = time;
					profiled_modes[mode][layout][start_pos].actions[num].time = time_now;
					//const double t = time;
					//profiled_modes[mode][layout][start_pos].times.push_back(t);
				}
			}
			else
			{
				profiled_modes[mode][layout][start_pos].exists = false; 
			}
		}
	}
}

	cmd_vel_modes.resize(max_mode_cmd_vel + 1);
    ROS_INFO("Loading cmd_vel params");
	for(int mode = 0; mode <= max_mode_cmd_vel; mode++)
	{
		cmd_vel_modes[mode].resize(4);
		for(int layout = 0; layout <= 1; layout++)
		{
            cmd_vel_modes[mode][layout+2].resize(max_start_pos_num + 1);
			cmd_vel_modes[mode][layout].resize(max_start_pos_num + 1);
			for(int start_pos = 0; start_pos <= max_start_pos_num; start_pos++)
			{
				std::string identifier("mode_"+std::to_string(mode)+"_layout_"+std::to_string(layout)+"_"+std::to_string(layout + 2)+"_start_"+std::to_string(start_pos));
                
                if(cmd_vel_data.getParam(identifier, cmd_vel_xml))
				{
					ROS_INFO_STREAM("cmd vel mode with identifier: " << identifier << " found cmd vel");
					const int num_segments = cmd_vel_xml.size();
                    cmd_vel_modes[mode][layout][start_pos].segments.resize(num_segments);
                    cmd_vel_modes[mode][layout+2][start_pos].segments.resize(num_segments);
					for(int num = 0; num<num_segments; num++) {
						XmlRpc::XmlRpcValue &segment = cmd_vel_xml[num];
						const double x = segment["x"];
						const double y = segment["y"];
						const double duration = segment["duration"];
                        cmd_vel_modes[mode][layout][start_pos].segments[num].x = x;
                        cmd_vel_modes[mode][layout][start_pos].segments[num].y = y;
                        cmd_vel_modes[mode][layout][start_pos].segments[num].duration = duration;
                        
                        cmd_vel_modes[mode][layout+2][start_pos].segments[num].x = x;
                        cmd_vel_modes[mode][layout+2][start_pos].segments[num].y = y;
                        cmd_vel_modes[mode][layout+2][start_pos].segments[num].duration = duration;
                    }
                    cmd_vel_modes[mode][layout][start_pos].exists = true; 
                    cmd_vel_modes[mode][layout+2][start_pos].exists = true; 
                    
                }
            }
        }
    }
    Modes all_modes;
    all_modes.profiled_modes = profiled_modes;
    all_modes.cmd_vel_modes = cmd_vel_modes;
	return all_modes;
}

bool generateTrajectory(std::vector<FullMode> &trajectory, const std::vector<int> &start_buffer_ids, const std::vector<bool> &generate, std::vector<std::vector<trajectory_msgs::JointTrajectory>> &trajectory_msgs_pass_out)
{
	ROS_ERROR("gen called");
	ROS_ERROR_STREAM("num traj msgs: " << trajectory_msgs_pass_out.size());
	trajectory_msgs_pass_out.resize(trajectory.size());


    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.wipe_all = false;
    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.brake = true;
    swerve_control_srv.request.run    = false;
    swerve_control_srv.request.change_queue   = false; 

	for(size_t k = 0; k < trajectory.size(); k++)
	{
        
		if(!generate[k]) continue;
		trajectory_msgs_pass_out[k].clear();
		auto_mode_status_vect[k] = 1;

		//ROS_ERROR("DO WE fail here");
		if(!trajectory[k].exists)
		{
			//TODO MAKE LIGHT GO RED ON DRIVERSTATION
			ROS_ERROR("auto mode/layout/start selected which wasn't found in the yaml");
            auto_mode_status_vect[k] = 0;
			return false;
		}
		ROS_WARN_STREAM("num srv msgs: " << trajectory[k].num_srv_msgs);
		for(size_t i = 0; i < trajectory[k].num_srv_msgs; i++)
		{
			//ROS_WARN_STREAM("i: " << i << " k: " << k);
			if (!point_gen.call(trajectory[k].srv_msgs[i]))
			{
				ROS_ERROR("point_gen call failed in autoInterpreterClient generateTrajectory()");
				return false;
			}
			talon_swerve_drive_controller::SwervePointSet temp_holder;
			temp_holder.dt = trajectory[k].srv_msgs[i].response.dt;
			temp_holder.points = trajectory[k].srv_msgs[i].response.points;
			temp_holder.slot = i + start_buffer_ids[k];
			swerve_control_srv.request.profiles.push_back(temp_holder);
					
			trajectory_msgs_pass_out[k].push_back(trajectory[k].srv_msgs[i].response.joint_trajectory);

	
		}
        ROS_ERROR("Succeeded in generate Trajectory");
		//ROS_ERROR_STREAM("num traj msgs sub msgs: " << trajectory_msgs_pass_out[k].size());
	}
    if (!swerve_control.call(swerve_control_srv))
	{
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient generateTrajectory()");
		return false;
	}
	return true;
}

bool generateCmdVelTrajectory(const CmdVelMode &segments)
{
    //ROS_ERROR_STREAM("In generate");
	if(!segments.exists)
	{
		//TODO MAKE LIGHT GO RED ON DRIVERSTATION
		ROS_ERROR("auto cmd_vel mode/layout/start selected which wasn't found in the yaml");
		return false;
	}
    
	return true;
}
bool generateCmdVel(const CmdVelMode &mode) {
    if(!mode.exists) {
        ROS_ERROR("auto mode/layout/start selected wasn't found in the yaml");
        return false;
    }
    return true;
}

std::string lower(const std::string &str) {
    std::string new_string;
    for(int i = 0; i<str.size(); i++) 
       new_string += tolower(str[i]); 
  
    return new_string;
}
bool queue_profile(std::vector<int> queue)
{
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.buffer = false;
    swerve_control_srv.request.run    = false;
    swerve_control_srv.request.wipe_all = false;
    swerve_control_srv.request.change_queue   = true; 
    swerve_control_srv.request.brake = false;
    for(size_t i = 0; i < queue.size(); i++)
	{
		swerve_control_srv.request.new_queue.push_back(queue[i]); 
    }
    if (!swerve_control.call(swerve_control_srv))
	{
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient queue_profile");
		return false;
	}
	ROS_WARN("why no true");
	return true;

}
bool runTrajectory(int slot) {
    //ROS_WARN("Run trajectory");
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.buffer = false;
    swerve_control_srv.request.run    = true;
    swerve_control_srv.request.brake = false;
    swerve_control_srv.request.wipe_all = false;
    swerve_control_srv.request.change_queue   = false; 
    swerve_control_srv.request.run_slot = slot;
    
    if (!swerve_control.call(swerve_control_srv))
	{
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient runTrajectory()");
		return false;
	}
	return true;
}

void match_data_cb(const ros_control_boilerplate::MatchSpecificData::ConstPtr &msg) {
    matchData.writeFromNonRT(MatchData(msg->isEnabled, msg->isAutonomous, msg->allianceData));

	// Not sure about the msg->isEnabled part - robot
	// will stop driving if disabled and it might be
	// nice for testing to continue the code for
	// debugging
	if (!(msg->isAutonomous && msg->isEnabled))
		exit_auto = true;

}

void queue_slot_cb(const std_msgs::UInt16::ConstPtr &msg) {
    queue_slot.writeFromNonRT(msg->data);

}



void auto_mode_cb(const ros_control_boilerplate::AutoMode::ConstPtr &msg) {
    autoMode.writeFromNonRT(AutoMode(msg->mode, msg->delays, msg->position));
}

bool check_action_completion(Action action, bool &intake_server_action, bool &robot_server_action, bool &timed_out) {
    switch(action) {
        case deploy_intake:
            return true;
        case undeploy_intake:
            return true;
        case intake_cube:
			if(robot_server_action || intake_server_action)
			{
				robot_server_action = true;
				intake_server_action = true;
				return true;		
			}
			if(ac->getState().isDone())
			{
				if(ac->getResult()->timed_out)
				{
					timed_out = true;
					return false;
				}	
				else
				{
					return true;
				}
			}
			else
			{
				return false;
			}
        case do_nothing:
            return true;
        case intake_no_arm:
            return true;
        case exchange_cube:
            return true;
        case default_config:
            return true;
        case intake_config:
            return true;
        case switch_config:
            return true;
        case low_scale_config:
            return true;
        case mid_scale_config:
            return true;
        case high_scale_config:
            return true;
        case over_back_config:
            return true;
        case custom_config:
            return true;
        case release_clamp:
            return true;
        case intake_open_down:
            return true;
        default:
            ROS_ERROR("Action not in list of actions");
			return true;
	}
}

bool call_action(Action action, const ActionSetpoint &action_setpoint) {
    ROS_WARN("%d", action);
    switch(action) {
        case do_nothing:
            break;
        case deploy_intake:
            deployIntake();
            break;
        case undeploy_intake:
            undeployIntake();
            break;
        case intake_cube:
            intakeCube(action_setpoint);
            break;
        case intake_no_arm:
            intakeNoArm();
            break;
        case exchange_cube:
            intakeOut();
            break;
        case default_config:
            defaultConfig();
            break;
        case intake_config:
            intakeConfig();
            break;
        case switch_config:
            switchConfig();
            break;
        case low_scale_config:
            lowScale();
            break;
        case mid_scale_config:
            midScale();
            break;
        case high_scale_config:
            highScale();
            break;
        case over_back_config:
            overBack();
            break;
        case custom_config:
            customConfig(action_setpoint);
            break;
        case release_clamp:
            releaseClamp();
            break;
		case intake_open_down:
			intakeOpenDown();
			break;
        default:
            ROS_ERROR("Action not in list of actions");

    }
}

void run_auto(int auto_select, int layout, int start_pos, double initial_delay, const std::vector<CmdVel> &segments) {
    exit_auto = false;

    clamp();
    if(auto_select == 0) {
        return;
    }

    ros::Rate r(10);
    double start_time = ros::Time::now().toSec();
    while(!exit_auto && ros::Time::now().toSec() < start_time + initial_delay) {
        r.sleep(); 
    }
    

    geometry_msgs::Twist vel;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    
    start_time = ros::Time::now().toSec();
    const size_t num_segments = segments.size();
    for(size_t num = 0; num<num_segments; num++) {
        const double segment_start_time = ros::Time::now().toSec();
        double curr_time = ros::Time::now().toSec(); //These two vars are the same..........
        while(curr_time < segment_start_time + segments[num].duration) { //HOW DOES THIS LOOP EVER EXIT!!
            if(curr_time > start_time + 1 && curr_time < start_time + 1 + r.expectedCycleTime().toSec()) {
                if(auto_select == 2) {
                    switchConfig();
                }
            }
            vel.linear.x = segments[num].x;        
            vel.linear.y = segments[num].y;        

            VelPub.publish(vel);
            r.sleep();
            curr_time = ros::Time::now().toSec();
        }
    }

    if(auto_select == 2) {
        double at_switch = ros::Time::now().toSec();
        while(!exit_auto && ros::Time::now().toSec() < at_switch + 1) {
            r.sleep(); 
        }
        releaseClamp();
    }
    parkingConfig();
}

void run_auto(int auto_select, int layout, int start_pos, double initial_delay, const FullMode &auto_run_data, std::vector<int> start_of_buffer_ids)
{
    //ROS_WARN("auto entered");
    ROS_WARN("auto_select:[%d], layout:[%d], start_pos:[%d]", auto_select, layout, start_pos);
    exit_auto = false;
    ros::Rate r(10);

    double start_time = ros::Time::now().toSec();
   

	robot_visualizer::ProfileFollower srv_viz_msg;
	srv_viz_msg.request.joint_trajectories = joint_trajectories[layout];

	srv_viz_msg.request.start_id = start_of_buffer_ids[layout];

	if(!VisualizeService.call(srv_viz_msg))
	{
		ROS_ERROR("failed to call viz srv");
	}
	else
	{
		ROS_ERROR("succeded in call to viz srv");
	}
 
    while(!exit_auto && ros::Time::now().toSec() < start_time + initial_delay) {
        r.sleep(); 
    }

    
	//while(!exit_auto && !runTrajectory(start_of_buffer_ids[layout])) //There is a better way
    //    r.sleep();
	
    start_time = ros::Time::now().toSec();
    size_t num_actions = auto_run_data.actions.size();
	size_t num_profs = auto_run_data.num_srv_msgs;
    ros::Rate action_rate(20);
		
	int queued = -1;
	std::vector<bool> finished;

	
    for(size_t num = 0; num<num_actions; num++)
	{
		finished.push_back(false);
	}
	std::vector<double> time_finished;

	
    for(size_t num = 0; num<num_actions; num++)
	{
		time_finished.push_back(DBL_MAX);
	}
    for(size_t num = 0; num<num_actions; num++) {

		const double action_start_time = ros::Time::now().toSec();
        //double curr_time = ros::Time::now().toSec();  //These two vars are the same..........
        
		

		bool dependencies_run =  false;

		while(/*curr_time*/ (ros::Time::now().toSec() < (start_time + auto_run_data.actions[num].time)) || (!dependencies_run ) && !exit_auto) { //HOW DOES THIS LOOP EVER EXIT!!! (other than if auto ends)
			
			bool intake_action_lib_later_write = false;
			bool robot_action_lib_later_write = false;
			bool timed_out = false;
			
			//ROS_WARN_STREAM("Time check: " << (ros::Time::now().toSec() < (action_start_time + auto_run_data.actions[num].time)) <<  " dependency check: " << 	dependencies_run);
				


			if(num != 0)
			{
				for(int i = num - 1; i >= 0; i--)
				{	
					if(finished[i]) continue;
					finished[i] = check_action_completion(auto_run_data.actions[i].action, intake_action_lib_later_write, robot_action_lib_later_write, timed_out);	
				time_finished[i] = ros::Time::now().toSec();	
					exit_auto = exit_auto || timed_out;	
				}
			}
			if(auto_run_data.actions[num].wait_action_id >=0)
			{
				dependencies_run = finished[auto_run_data.actions[num].wait_action_id] && ros::Time::now().toSec() - time_finished[auto_run_data.actions[num].wait_action_id] > auto_run_data.actions[num].wait_action_time;

			}
			else
			{
				dependencies_run = true; 	
			} 
			if(auto_run_data.actions[num].wait_action_id >=0)
			{
				dependencies_run = dependencies_run && *(queue_slot.readFromRT()) >= auto_run_data.actions[num].wait_profile_id; //TODO: add support a wait here maybe
			}

			//ROS_WARN_STREAM("prof count: " << num_profs);
			for(int i = 0; i < num_profs; i++)
			{
				//ROS_INFO_STREAM("start loop");
				//ROS_INFO_STREAM("wait id: " << auto_run_data.wait_ids[i]);
				if(i <= queued) continue;
				//ROS_INFO_STREAM("running loop still");
				//ROS_WARN_STREAM("here " << i << " and " << auto_run_data.wait_ids[i]);	
				//ROS_INFO_STREAM("hypothetical id: " << auto_run_data.wait_ids[i]);
				if(auto_run_data.wait_ids[i] != -1) ROS_WARN_STREAM(" hypothetical finish " << finished[auto_run_data.wait_ids[i]]);

				if(auto_run_data.wait_ids[i] == -1 || finished[auto_run_data.wait_ids[i]])
				{
					
					//ROS_WARN_STREAM("Queued" << queued);
					queued = i;
					if(i!=0)
					{
						std::vector<int> queue;
						for(int k = 1; k < queued + 1; k++)
						{
							queue.push_back(start_of_buffer_ids[layout] + k); 	
							
						}
						//ROS_WARN_STREAM("trying to queue");
						if(!queue_profile(queue))
						{	
									
							ROS_WARN("queue fail");
							queued -= 1;
							action_rate.sleep();
							break;
							//If we fail try again
						}	
					}
					else if(!runTrajectory(start_of_buffer_ids[layout]))
					{
						ROS_WARN("runTrajectory fail");
						queued -= 1;
						action_rate.sleep();
						break;
						//If we fail try again
					}
				}
				else
				{
					break;
				}
			}
            action_rate.sleep();
	
		}
		ROS_ERROR_STREAM("hypothetically running action: " << num);	
        call_action(auto_run_data.actions[num].action, auto_run_data.actions[num].action_setpoint);
	}
	
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Auto_state_subscriber");
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");

    ros::NodeHandle n_params_behaviors(n, "auto_params");
    ros::NodeHandle n_params_cmd_vel(n, "cmd_vel_params");
    
    int wait_for_match_data;
    int num_cmd_vel_modes;
   
    if (!n_params.getParam("wait_for_match_data", wait_for_match_data))
		ROS_ERROR("Didn't read param wait_match_data in autoInterpreterClient");
    if (!n_params.getParam("high_scale_config_y", high_scale_config_y))
		ROS_ERROR("Didn't read param high_scale_config_y in autoInterpreterClient");
    if (!n_params.getParam("mid_scale_config_x", mid_scale_config_x))
		ROS_ERROR("Didn't read param mid_scale_config_x in autoInterpreterClient");
    if (!n_params.getParam("mid_scale_config_y", mid_scale_config_y))
		ROS_ERROR("Didn't read param mid_scale_config_y in autoInterpreterClient");
    if (!n_params.getParam("low_scale_config_x", low_scale_config_x))
		ROS_ERROR("Didn't read param low_scale_config_x in autoInterpreterClient");
    if (!n_params.getParam("low_scale_config_y", low_scale_config_y))
		ROS_ERROR("Didn't read param low_scale_config_y in autoInterpreterClient");
    if (!n_params.getParam("switch_config_x", switch_config_x))
		ROS_ERROR("Didn't read param switch_config_x in autoInterpreterClient");
    if (!n_params.getParam("switch_config_y", switch_config_y))
		ROS_ERROR("Didn't read param switch_config_y in autoInterpreterClient");
    if (!n_params.getParam("exchange_config_x", exchange_config_x))
		ROS_ERROR("Didn't read param exchange_config_x in autoInterpreterClient");
    if (!n_params.getParam("exchange_config_y", exchange_config_y))
		ROS_ERROR("Didn't read param exchange_config_y in autoInterpreterClient");
    if (!n_params.getParam("intake_ready_to_drop_x", intake_ready_to_drop_x))
		ROS_ERROR("Didn't read param intake_ready_to_drop_x in autoInterpreterClient");
    if (!n_params.getParam("intake_ready_to_drop_y", intake_ready_to_drop_y))
		ROS_ERROR("Didn't read param intake_ready_to_drop_y in autoInterpreterClient");
    if (!n_params.getParam("intake_ready_to_drop_up_or_down", intake_ready_to_drop_up_or_down))
		ROS_ERROR("Didn't read param intake_ready_to_drop_up_or_down in autoInterpreterClient");
    if (!n_params.getParam("over_back_x", over_back_x))
		ROS_ERROR("Didn't read param over_back_x in autoInterpreterClient");
    if (!n_params.getParam("over_back_y", over_back_y))
		ROS_ERROR("Didn't read param over_back_y in autoInterpreterClient");
    if (!n_params.getParam("default_x", default_x))
		ROS_ERROR("Didn't read param default_x in autoInterpreterClient");
    if (!n_params.getParam("default_y", default_y))
		ROS_ERROR("Didn't read param default_y in autoInterpreterClient");
	if (!n_params.getParam("default_up_or_down", default_up_or_down))
		ROS_ERROR("Could not read default_up_or_down");


	int num_profile_slots;
	
    if (!n_params_behaviors.getParam("num_profile_slots", num_profile_slots))
		ROS_ERROR("Didn't read param num_profile_slots");

	std::vector<int> start_of_buffer_ids; 
	int iterator = floor(num_profile_slots / 4);
	start_of_buffer_ids.push_back(0);
	start_of_buffer_ids.push_back(iterator);
	start_of_buffer_ids.push_back(iterator*2);
	start_of_buffer_ids.push_back(iterator*3);

    int max_num_mode;
    int max_num_cmd_vel;
    int max_num_start;
    int max_num_wait_for_action;

    if (!n_params_behaviors.getParam("max_num_mode", max_num_mode))
		ROS_ERROR("Didn't read param max_num_mode in autoInterpreterClient");
    if (!n_params_behaviors.getParam("max_num_start", max_num_start))
		ROS_ERROR("Didn't read param max_num_start in autoInterpreterClient");
    if (!n_params_behaviors.getParam("max_num_wait_for_action", max_num_wait_for_action))
		ROS_ERROR("Didn't read param max_num_wait_for_action in autoInterpreterClient");
    
    if (!n_params_cmd_vel.getParam("max_num_mode", max_num_cmd_vel))
		ROS_ERROR("Didn't read cmd_vel param max_num_mode in autoInterpreterClient");
    if (!n_params_cmd_vel.getParam("num_cmd_vel_modes", num_cmd_vel_modes))
		ROS_ERROR("Didn't read param num_cmd_vel_modes in autoInterpreterClient");
	// Load trajectories before callbacks which use them can
	// start
    //ros::Duration(20).sleep();
	Modes all_modes = load_all_trajectories(max_num_mode, max_num_cmd_vel, max_num_start, iterator, n_params_behaviors, n_params_cmd_vel);
    ModeList profiled_modes = all_modes.profiled_modes;
    CmdVelList cmd_vel_modes = all_modes.cmd_vel_modes;

    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::RobotAction>>("auto_interpreter_server", true);
    ac_intake = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("auto_interpreter_server_intake", true);
    ROS_WARN("here 567890");
	ac->waitForServer(); 
	ac_intake->waitForServer(); 

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);

    IntakeService = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
    ElevatorService = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
    ClampService = n.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
    BrakeService = n.serviceClient<std_srvs::Empty>("/frcrobot/swerve_drive_controller/brake", false, service_connection_header);
	VisualizeService = n.serviceClient<robot_visualizer::ProfileFollower>("/frcrobot/visualize_auto", false, service_connection_header);    

    VelPub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);
    auto_state_0 = n.advertise<std_msgs::Float64>("/frcrobot/auto_state_controller_0/command", 1);
    auto_state_1 = n.advertise<std_msgs::Float64>("/frcrobot/auto_state_controller_1/command", 1);
    auto_state_2 = n.advertise<std_msgs::Float64>("/frcrobot/auto_state_controller_2/command", 1);
    auto_state_3 = n.advertise<std_msgs::Float64>("/frcrobot/auto_state_controller_3/command", 1);

    std_msgs::Float64 state_0;
    std_msgs::Float64 state_1;
    std_msgs::Float64 state_2;
    std_msgs::Float64 state_3;
    
    ros::Subscriber auto_mode_sub = n.subscribe("autonomous_mode", 1, &auto_mode_cb);
    ros::Subscriber slot_sub = n.subscribe("profile_queue_num", 1, &queue_slot_cb);
    ros::Subscriber match_data_sub = n.subscribe("match_data", 1, &match_data_cb);

	// Kick off 2 threads to process messages
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
    ROS_WARN("Auto Client loaded");
    //ros::Duration(4).sleep();

    //ROS_WARN("post sleep");
   
	/*

	std::vector<bool> test_modes = {true, false, false, false};
	std::vector<int> test_modes_slot = {0, 1, 2, 3};
	std::vector<FullMode> test_mode_gen;
	test_mode_gen.push_back(profiled_modes[0][0][0]);
	test_mode_gen.push_back(profiled_modes[0][1][0]);
	test_mode_gen.push_back(profiled_modes[0][2][0]);
	test_mode_gen.push_back(profiled_modes[0][3][0]);

	*/
 
    /*---------------------------- JUST FOR TESTING ------------------------------------ */
    //generateTrajectory(test_mode_gen, test_modes_slot = {0, 1, 2, 3}, test_modes = {true, true, true, true});
    //ROS_WARN("Auto Client loaded");
    //ros::Duration(30).sleep();
    //ROS_WARN("post sleep");
	//return 1; 
    /*---------------------------- JUST FOR TESTING ------------------------------------ */
    //generateTrajectory(profiled_modes[3][3][2]);
    /*---------------------------- JUST FOR TESTING ------------------------------------ */

	//return 1;

    //ROS_WARN("SUCCESS IN autoInterpreterClient.cpp");
    ros::Rate r(10);



	ros::service::waitForService("/frcrobot/swerve_drive_controller/run_profile", 60);
    ros::Duration(4).sleep();



    while(ros::ok()) {
        ROS_WARN("running");
        std::vector<bool> generate_for_this;
        std::vector<bool> mode_buffered;
        mode_buffered.resize(4);
        generate_for_this.resize(4);
        double auto_start_time = DBL_MAX;
        //std::vector<bool> generated_vect = {false, false, false, false};
        std::vector<int> auto_mode_vect = {-1, -1, -1, -1};
        std::vector<double> delays_vect = {0, 0, 0, 0};
        int start_pos = 0;
		int last_start_pos = 0;
        bool match_data_received = false;
        //int auto_mode = -1;
        int layout = 0;
        bool in_auto = false;
		bool last_in_auto = false;
        bool end_auto = false;
        //bool run_regardless = false;
        bool in_teleop = false;
		const FullMode empty_full_mode;
        while(!in_teleop && !end_auto) {
            ////ROS_ERROR("In auto loop");

            MatchData match_data = *(matchData.readFromRT());
            AutoMode auto_mode_data = *(autoMode.readFromRT());

			//ROS_INFO_STREAM("in auto: " << match_data.isAutonomous_ << " enabled? " << 

			if(match_data.isAutonomous_ && !match_data.isEnabled_)
			{
				static int reset_counter = 0;
				reset_counter ++;
				if(reset_counter % 10 == 1)
				{
					//Set up for auto
					parkingConfig();
					startMatchConfig();
					clamp();
					elevator_controller::Intake srv;
					srv.request.spring_state = 2; //soft_in
					srv.request.power=0;
                    srv.request.other_power=0;
					srv.request.just_override_power = false;
					srv.request.up = true;
					if(!IntakeService.call(srv))
					{
						ROS_ERROR("Service call failed : IntakeService in deployIntake");
					}
				}

            
			}
			if(!in_auto && !last_in_auto) { //accept changes to chosen auto_modes until we receive match data or auto starts
                start_pos = auto_mode_data.start_pos_;
                ////ROS_INFO("No match data and not in auto");
                //loop through auto_mode data 
                //generate trajectories for all changed modes
                //bool any_change = false;
				std::vector<FullMode> out_to_generate;
				bool generate = false;	
				for(int i = 0; i<4; i++) {
					if ((auto_mode_data.modes_[i] > num_cmd_vel_modes-1) &&
                        ((((auto_mode_data.modes_[i] != auto_mode_vect[i]))) || (auto_mode_data.start_pos_ != last_start_pos)))
                    {
						ROS_ERROR_STREAM("actual mode: " << auto_mode_data.modes_[i] - num_cmd_vel_modes << " layout: " << i << " start: " << auto_mode_data.start_pos_); 

                        out_to_generate.push_back(profiled_modes[auto_mode_data.modes_[i] - num_cmd_vel_modes][i][auto_mode_data.start_pos_]);
						generate_for_this[i] = true;	
						generate = true;
                        auto_mode_status_vect[i]=1;
						//any_change = true;
                    }
                    else if (auto_mode_data.modes_[i] <= num_cmd_vel_modes-1 && (auto_mode_data.modes_[i] >= 0) &&
                        ((auto_mode_data.modes_[i] != auto_mode_vect[i]) || (auto_mode_data.start_pos_ != last_start_pos)))
                    {
                        out_to_generate.push_back(empty_full_mode);
						generate_for_this[i] = false;	
                        if(generateCmdVelTrajectory(cmd_vel_modes[auto_mode_data.modes_[i]][i][auto_mode_data.start_pos_])) {
							mode_buffered[i] = true;
                            auto_mode_status_vect[i]=1;

                            auto_mode_vect[i] = auto_mode_data.modes_[i];
                            delays_vect[i] = auto_mode_data.delays_[i];

                            //generated_vect[i] = true;
                            //ROS_ERROR_STREAM("Generating Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
                        }
                        else {
							mode_buffered[i] = false;
                            auto_mode_vect[i] = 0;
                            delays_vect[i] = 0;
                            //generated_vect[i] = false;
                            //ROS_ERROR_STREAM("Invalid Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
                        }
                    }
                    else {
                        out_to_generate.push_back(empty_full_mode);
						generate_for_this[i] = false;	
                    }
                }
				if(generate)
				{		
					if(generateTrajectory(out_to_generate, start_of_buffer_ids, generate_for_this, joint_trajectories)) {
						for(int i = 0; i<4; i++) {
							if(!generate_for_this[i]) continue;
							//ROS_ERROR_STREAM("4");
							mode_buffered[i] = true;
							//generated_vect[i] = true;
							auto_mode_vect[i] = auto_mode_data.modes_[i];
							delays_vect[i] = auto_mode_data.delays_[i];
							//ROS_ERROR_STREAM("Generating Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
							ROS_ERROR_STREAM("Sizing: " << joint_trajectories[i].size());
						}

					}
					else {
						for(int i = 0; i<4; i++) {
							if(!generate_for_this[i]) continue;
							mode_buffered[i] = false;
							auto_mode_vect[i] = 0;
							delays_vect[i] = 0;
							//start_pos = -1;
							//generated_vect[i] = false;
							//ROS_ERROR_STREAM("Invalid Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
						}
					}
				}
				last_start_pos = start_pos;
            }

            if(in_auto && match_data.isAutonomous_ == false) {
                //ROS_ERROR_STREAM("Leaving Autonomous to teleop");
                in_teleop = true;
            }
            if (match_data.alliance_data_ != "") {
                //ROS_INFO("Receiving alliance data");
                match_data_received = true;
                if(lower(match_data.alliance_data_)=="rlr") {
                    layout = 0;
                }
                else if(lower(match_data.alliance_data_)=="lrl") {
                    layout = 1;
                }
                else if(lower(match_data.alliance_data_)=="rrr") {
                    layout = 2;
                }
                else if(lower(match_data.alliance_data_) =="lll") {
                    layout = 3;
                } 
                else {
                    ROS_ERROR("Invalid Alliance Data");
                    match_data_received = false;
                }
            }
            else {
                ////ROS_INFO("No alliance data");
                match_data_received = false;
            }
            if((!match_data_received || !mode_buffered[layout] )&& ros::Time::now().toSec() > auto_start_time + wait_for_match_data) { //if match data isn't found after 2 seconds of auto or nothing is buffered starting run default auto
                //ROS_INFO("In first two seconds of auto with no match data");
                layout = 0;
                auto_mode_vect[layout] = 1; //default auto: cross baseline forward
                //generated_vect[layout] = true;
                match_data_received = true;
				//run_regardless = true;
            }
            if(!in_auto) { //check for auto to start and set a start time
                //ROS_INFO("Not in auto yet");

                if(match_data.isAutonomous_ && match_data.isEnabled_) {
                    //ROS_ERROR_STREAM("Entering Auto");
                    in_auto = true;
                    auto_start_time = ros::Time::now().toSec();
                }
            }
            
			if(in_auto) {
                if(!last_in_auto)
				{
					//Set up for auto
					startMatchConfig();
					clamp();
					elevator_controller::Intake srv;
					srv.request.spring_state = 2; //soft_in
					srv.request.power=0;
                    srv.request.other_power=0;
					srv.request.just_override_power = false;
					srv.request.up = true;
					if(!IntakeService.call(srv))
					{
						ROS_ERROR("Service call failed : IntakeService in deployIntake");
					}
				}			




				//ROS_INFO("In auto");
                if(!match_data.isAutonomous_ || !match_data.isEnabled_) {
                    //ROS_ERROR_STREAM("Disabled in Auto");
                    in_auto = false;
                    auto_start_time = DBL_MAX;
                }
            }

			last_in_auto = in_auto;

            if(in_auto && mode_buffered[layout] && match_data_received) { //if in auto with mode buffered run it
                //ROS_INFO("Match data received and auto buffered");
                if(auto_mode_vect[layout] > num_cmd_vel_modes-1) {
                    run_auto(auto_mode_vect[layout], layout, start_pos, 
                             delays_vect[layout], profiled_modes[auto_mode_vect[layout]-num_cmd_vel_modes][layout][start_pos], start_of_buffer_ids);
                }




                else if(auto_mode_vect[layout] >= 0) {
                    run_auto(auto_mode_vect[layout], layout, start_pos, 
                             delays_vect[layout], cmd_vel_modes[auto_mode_vect[layout]][layout][start_pos].segments);
                    
                }
                //ROS_ERROR_STREAM("Running Auto");
                // Auto finished, either by finishing the requested path
                // or by match data reporting not in autonomous
                in_auto = false;
                end_auto = true;
            }

            /*
            auto_mode_status_vect[0] = 1;
            auto_mode_status_vect[1] = 1;
            auto_mode_status_vect[2] = 1;
            auto_mode_status_vect[3] = 1;
            */
            state_0.data = auto_mode_status_vect[0];
            state_1.data = auto_mode_status_vect[1];
            state_2.data = auto_mode_status_vect[2];
            state_3.data = auto_mode_status_vect[3];
            
            auto_state_0.publish(state_0);
            auto_state_1.publish(state_1);
            auto_state_2.publish(state_2);
            auto_state_3.publish(state_3);
            r.sleep();
        
		}

        /* ------------- For testing wait until !in_teleop to loop back to beginning -----------------------*/
        ros::Rate slow(.5);
        //ROS_WARN("Exited Auto loop");
        while(end_auto) {
            //ROS_INFO("Between auto and teleop or robot disabled");
            MatchData match_data = *(matchData.readFromRT());
            if(!match_data.isAutonomous_ || !match_data.isEnabled_) {
                end_auto = false;
                in_teleop = !match_data.isAutonomous_ && match_data.isEnabled_;
            }
			else
				slow.sleep(); //I think you want this.....
        }
        while(in_teleop) {
            ROS_INFO("Exited auto into teleop");
            MatchData match_data = *(matchData.readFromRT());
            if(!match_data.isEnabled_ || match_data.isAutonomous_) {
                in_teleop = false;
            }
			else
				slow.sleep(); 
        }
    }
    ac.reset();
    ac_intake.reset();
    return 0;
}
