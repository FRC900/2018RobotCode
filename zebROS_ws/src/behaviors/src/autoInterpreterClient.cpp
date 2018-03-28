#include <atomic>

#include <behaviors/autoInterpreterClient.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/RobotAction.h>
#include <realtime_tools/realtime_buffer.h>


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
static ros::Publisher VelPub;

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
realtime_tools::RealtimeBuffer<AutoMode> autoMode;



bool defaultConfig(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = default_x;
    srv.request.y = default_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    srv.request.put_cube_in_intake = false;
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
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in intakeConfig");
		return false;
	}
	return true;
}

bool intakeCube(void) {
	behaviors::RobotGoal goal;

    ROS_WARN("intaking cube");
    goal.IntakeCube = true;
    goal.MoveToIntakeConfig = false;
    goal.x = default_x;
    goal.y = default_y;
    goal.up_or_down = default_up_or_down;
    goal.override_pos_limits = false; //TODO config these
    goal.dist_tolerance = .5; //TODO config these
    goal.x_tolerance = .5; //TODO config these
    goal.y_tolerance = .5; //TODO config these
    goal.time_out = 3; //TODO config these

    ac->sendGoal(goal);
    
}
bool intakeNoArm(void) {
	behaviors::IntakeGoal goal;

    ROS_WARN("intaking cube");
    goal.IntakeCube = true;
    goal.time_out = 10; //TODO config this

    ac_intake->sendGoal(goal);
    
}
bool switchConfig(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = switch_config_x;
    srv.request.y = switch_config_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
    srv.request.put_cube_in_intake = false;
    srv.request.override_sensor_limits = false;
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
    srv.request.power=0;
    if (!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in intakeStop");
		return false;
	}
	return true;
}
bool releaseIntake(void) {
	elevator_controller::Intake srv;
    srv.request.spring_state = 1; //hard out
    srv.request.power = 0;
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
	srv.request.spring_state = 2; //soft-in
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
	srv.request.spring_state = 2; //soft-in
    srv.request.up = 0; //down
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
	srv.request.spring_state = 2; //soft-in
    srv.request.up = true;
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
	//ROS_WARN("Braking");
	if (!BrakeService.call(empty))
	{
		ROS_ERROR("Service call failed : BrakeService in parkingConfig");
		return false;
	}
	return true;
}


Modes load_all_trajectories(int max_mode_num, int max_mode_cmd_vel, int max_start_pos_num, ros::NodeHandle &auto_data, ros::NodeHandle &cmd_vel_data)
{
	XmlRpc::XmlRpcValue mode_xml;
	XmlRpc::XmlRpcValue actions_xml;

	XmlRpc::XmlRpcValue cmd_vel_xml;

	mode_list profiled_modes;
    cmd_vel_list cmd_vel_modes;

	profiled_modes.resize(max_mode_num + 1);
	for(int mode = 0; mode <= max_mode_num; mode++)
	{
		profiled_modes[mode].resize(4);
		for(int layout = 0; layout <= 3; layout++)
		{
			profiled_modes[mode][layout].resize(max_start_pos_num + 1);
			for(int start_pos = 0; start_pos <= max_start_pos_num; start_pos++)
			{
				
				std::string identifier("mode_"+std::to_string(mode)+"_layout_"+
				std::to_string(layout)+"_start_"+std::to_string(start_pos));
                
                if(auto_data.getParam(identifier, mode_xml))
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
						profiled_modes[mode][layout][start_pos].srv_msg.request.x_coefs.push_back(x_coefs);
						profiled_modes[mode][layout][start_pos].srv_msg.request.y_coefs.push_back(y_coefs);
						profiled_modes[mode][layout][start_pos].srv_msg.request.orient_coefs.push_back(orient_coefs);
						profiled_modes[mode][layout][start_pos].srv_msg.request.end_points.push_back(num+1);
					}
					profiled_modes[mode][layout][start_pos].srv_msg.request.initial_v = 0; 
					profiled_modes[mode][layout][start_pos].srv_msg.request.final_v = 0; 
					profiled_modes[mode][layout][start_pos].exists = true; 

					XmlRpc::XmlRpcValue group_xml;
					if(auto_data.getParam(identifier + "_spline_group", group_xml))
					{
						//ROS_INFO_STREAM("Custom grouping for identifier: " << identifier << " found");
						for(int i = 0; i < group_xml.size(); i++)
						{	
							profiled_modes[mode][layout][start_pos].srv_msg.request.spline_groups.push_back(group_xml[i]);
						}
						XmlRpc::XmlRpcValue wait_xml;
						if(auto_data.getParam(identifier + "_waits", wait_xml))
						{
							//ROS_INFO_STREAM("Custom waits for identifier: " << identifier << " found");
							for(int i = 0; i < group_xml.size(); i++)
							{
								profiled_modes[mode][layout][start_pos].srv_msg.request.wait_before_group.push_back(wait_xml[i]);
							}
						}
						else
						{
							for(int i = 0; i < group_xml.size(); i++)
							{
								profiled_modes[mode][layout][start_pos].srv_msg.request.wait_before_group.push_back(.16);
							}
						}
						XmlRpc::XmlRpcValue shift_xml;
						if(auto_data.getParam(identifier + "_t_shifts", shift_xml))
						{
							ROS_INFO_STREAM("Custom shifts for identifier: " << identifier << " found");
							for(int i = 0; i < group_xml.size(); i++)
							{
								profiled_modes[mode][layout][start_pos].srv_msg.request.t_shift.push_back(shift_xml[i]);
							}
						}
						else
						{
							for(int i = 0; i < group_xml.size(); i++)
							{
								profiled_modes[mode][layout][start_pos].srv_msg.request.t_shift.push_back(0);
							}
						}
						XmlRpc::XmlRpcValue flip_xml;
						if(auto_data.getParam(identifier + "_flips", flip_xml))
						{
							ROS_INFO_STREAM("Custom flips for identifier: " << identifier << " found");
							for(int i = 0; i < group_xml.size(); i++)
							{
								profiled_modes[mode][layout][start_pos].srv_msg.request.flip.push_back(flip_xml[i]);
							}
						}
						else
						{
							for(int i = 0; i < group_xml.size(); i++)
							{
								profiled_modes[mode][layout][start_pos].srv_msg.request.flip.push_back(false);
							}
						}
					}
					else
					{
							profiled_modes[mode][layout][start_pos].srv_msg.request.flip.push_back(false);
							profiled_modes[mode][layout][start_pos].srv_msg.request.spline_groups.push_back(num_splines);
							profiled_modes[mode][layout][start_pos].srv_msg.request.wait_before_group.push_back(.16);
							profiled_modes[mode][layout][start_pos].srv_msg.request.t_shift.push_back(0);
					}
				}


				if(auto_data.getParam(identifier+ "_actions", actions_xml))
				{
					ROS_INFO_STREAM("Auto mode actions with identifier: " << identifier+"_actions" << " found");
					const int num_actions = actions_xml.size();
                    profiled_modes[mode][layout][start_pos].actions.resize(num_actions);
					for(int num = 0; num<num_actions; num++) {
						XmlRpc::XmlRpcValue &action = actions_xml[num];
						XmlRpc::XmlRpcValue &time = action["time"];
						XmlRpc::XmlRpcValue &actions = action["actions"];
						//XmlRpc::XmlRpcValue &time = spline["time"];
                        const double num_actions_now = actions.size();
						for(int i = 0; i<num_actions_now; i++) {
                            const std::string action_now = actions[num];
                            //profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(action_now);
                            if(action_now == "deploy_intake") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(deploy_intake);
                            }
                            else if(action_now == "undeploy_intake") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(undeploy_intake);
                            }
                            else if(action_now == "intake_cube") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(intake_cube);
                            }
                            else if(action_now == "exchange_cube") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(exchange_cube);
                            }
                            else if(action_now == "default_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(default_config);
                            }
                            else if(action_now == "intake_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(intake_config);
                            }
                            else if(action_now == "exchange_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(exchange_config);
                            }
                            else if(action_now == "switch_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(switch_config);
                            }
                            else if(action_now == "low_scale_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(low_scale_config);
                            }
                            else if(action_now == "mid_scale_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(mid_scale_config);
                            }
                            else if(action_now == "high_scale_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(high_scale_config);
                            }
                            else if(action_now == "over_back_config") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(over_back_config);
                            }
                            else if(action_now == "release_clamp") {
                                profiled_modes[mode][layout][start_pos].actions[num].actions.push_back(release_clamp);
                            }
						}
                        const double time_now = time;
                        profiled_modes[mode][layout][start_pos].actions[num].time = time_now;
						//const double t = time;
						//profiled_modes[mode][layout][start_pos].times.push_back(t);
					}
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

bool generateTrajectory(full_mode &trajectory) 
{
	ROS_ERROR("DO WE fail here");
	if(!trajectory.exists)
	{
		//TODO MAKE LIGHT GO RED ON DRIVERSTATION
		ROS_ERROR("auto mode/layout/start selected which wasn't found in the yaml");
		return false;
	}

	if (!point_gen.call(trajectory.srv_msg))
	{
		ROS_ERROR("point_gen call failed in autoInterpreterClient generateTrajectory()");
		return false;
	}
	if (!bufferTrajectory(trajectory.srv_msg.response))
	{
		ROS_ERROR("bufferTrajectory call failed in autoInterpreterClient generateTrajectory()");
		return false;
	}
	return true;
}

bool generateTrajectory(cmd_vel_mode &segments) 
{
	if(!segments.exists)
	{
		//TODO MAKE LIGHT GO RED ON DRIVERSTATION
		ROS_ERROR("auto cmd_vel mode/layout/start selected which wasn't found in the yaml");
		return false;
	}
    
	return true;
}
bool generateCmdVel(cmd_vel_mode &mode) {
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

bool bufferTrajectory(const swerve_point_generator::FullGenCoefs::Response &traj)
{
    //ROS_INFO_STREAM("Buffer trajectory num_points: " << traj.points.size() << " dt: " << traj.dt);
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.points = traj.points;
    swerve_control_srv.request.dt = traj.dt;
    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.run    = false;
    swerve_control_srv.request.slot   = 0; //TODO fix
    
    if (!swerve_control.call(swerve_control_srv))
	{
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient bufferTrajectory()");
		return false;
	}
	return true;
}

bool runTrajectory(void) {
    //ROS_WARN("Run trajectory");
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.buffer = false;
    swerve_control_srv.request.run    = true;
    swerve_control_srv.request.slot   = 0; //TODO fix
    
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

void auto_mode_cb(const ros_control_boilerplate::AutoMode::ConstPtr &msg) {
    autoMode.writeFromNonRT(AutoMode(msg->mode, msg->delays, msg->position));
}

bool call_action(int action) {
    switch(action) {
        case 0:
            deployIntake();
            break;
        case 1:
            undeployIntake();
            break;
        case 2:
            intakeCube();
            break;
        case 3:
            intakeNoArm();
            break;
        case 4:
            intakeOut();
            break;
        case 5:
            defaultConfig();
            break;
        case 6:
            intakeConfig();
            break;
        case 7:
            switchConfig();
            break;
        case 8:
            lowScale();
            break;
        case 9:
            midScale();
            break;
        case 10:
            highScale();
            break;
        case 11:
            overBack();
            break;
        case 12:
            releaseClamp();
            break;
        default:
            ROS_ERROR("Action not in list of actions");

    }
}


void run_auto(int auto_select, int layout, int start_pos, double initial_delay, const std::vector<cmd_vel_struct> &segments) {
    ROS_WARN("auto entered");
    exit_auto = false;
    ros::Rate r(10);
    double start_time = ros::Time::now().toSec();
    
    while(!exit_auto && ros::Time::now().toSec() < start_time + initial_delay) {
        r.sleep(); 
    }

    start_time = ros::Time::now().toSec();
    int num_segments = segments.size();
    
    if(auto_select == 0) {
        return;
    }

    geometry_msgs::Twist vel;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    for(int num = 0; num<num_segments; num++) {
        ROS_WARN("auto segment entered");
        double segment_start_time = ros::Time::now().toSec();
        double curr_time = ros::Time::now().toSec();
        while(curr_time < segment_start_time + segments[num].duration) {
            vel.linear.x = segments[num].x;        
            vel.linear.y = segments[num].y;        

            VelPub.publish(vel);
            r.sleep();
            curr_time = ros::Time::now().toSec();
        }
    }
    if(auto_select == 2) {
        releaseClamp();
    }
    parkingConfig();
}

void run_auto(int auto_select, int layout, int start_pos, double initial_delay, const std::vector<action_struct> &actions) {
    //ROS_WARN("auto entered");
    exit_auto = false;
    ros::Rate r(10);

    double start_time = ros::Time::now().toSec();
    
    while(!exit_auto && ros::Time::now().toSec() < start_time + initial_delay) {
        r.sleep(); 
    }

    while(!exit_auto && !runTrajectory())
        r.sleep();

    start_time = ros::Time::now().toSec();
    int num_actions = actions.size();

    ros::Rate action_rate(20);
    for(int num = 0; num<num_actions; num++) {
        double action_start_time = ros::Time::now().toSec();
        double curr_time = ros::Time::now().toSec();
        while(curr_time < action_start_time + actions[num].time && !exit_auto) {
            action_rate.sleep();
        }
        int num_actions_now = actions[num].actions.size();
        for(int i = 0; i<num_actions_now; i++) {
            call_action(actions[num].actions[i]);
        }
        
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
    if (!n_params.getParam("num_cmd_vel_modes", num_cmd_vel_modes))
		ROS_ERROR("Didn't read param num_cmd_vel_modes in autoInterpreterClient");
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


    int max_num_mode;
    int max_num_cmd_vel;
    int max_num_start;

    if (!n_params_behaviors.getParam("max_num_mode", max_num_mode))
		ROS_ERROR("Didn't read param max_num_mode in autoInterpreterClient");
    if (!n_params_behaviors.getParam("max_num_start", max_num_start))
		ROS_ERROR("Didn't read param max_num_start in autoInterpreterClient");
    
    if (!n_params_cmd_vel.getParam("max_num_mode", max_num_cmd_vel))
		ROS_ERROR("Didn't read cmd_vel param max_num_mode in autoInterpreterClient");
	// Load trajectories before callbacks which use them can
	// start
    ROS_ERROR("Here");
    ros::Duration(20).sleep();
    ROS_ERROR("Here1");
	Modes all_modes = load_all_trajectories(max_num_mode, max_num_cmd_vel, max_num_start, n_params_behaviors, n_params_cmd_vel);
    ROS_ERROR("Here2");
    ROS_WARN("Alls Here");
    mode_list profiled_modes = all_modes.profiled_modes;
    cmd_vel_list cmd_vel_modes = all_modes.cmd_vel_modes;

    ROS_ERROR("Here3");
    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::RobotAction>>("auto_interpreter_server", true);
    ROS_WARN("here 567890");
	ac->waitForServer(); 
    ROS_ERROR("Here4");

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", false, service_connection_header);
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);

    ROS_ERROR("Here5");
    IntakeService = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
    ElevatorService = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
    ClampService = n.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
    BrakeService = n.serviceClient<std_srvs::Empty>("/frcrobot/talon_swerve_drive_controller/brake", false, service_connection_header);
    
    VelPub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);

    ros::Subscriber auto_mode_sub = n.subscribe("autonomous_mode", 1, &auto_mode_cb);
    ros::Subscriber match_data_sub = n.subscribe("match_data", 1, &match_data_cb);

	// Kick off 2 threads to process messages
    ROS_ERROR("Here6");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
    ROS_WARN("Auto Client loaded");

    ROS_WARN("post sleep");
    
    /*---------------------------- JUST FOR TESTING ------------------------------------ */
    generateTrajectory(profiled_modes[3][3][2]);
    //ROS_WARN("Auto Client loaded");
    //ros::Duration(30).sleep();
    //ROS_WARN("post sleep");
    
    /*---------------------------- JUST FOR TESTING ------------------------------------ */
    //generateTrajectory(profiled_modes[3][3][2]);
    /*---------------------------- JUST FOR TESTING ------------------------------------ */

    //ROS_WARN("SUCCESS IN autoInterpreterClient.cpp");
    ros::Rate r(10);

    ROS_ERROR("Here7");
    while(ros::ok()) {
        ROS_WARN("running");
        double auto_start_time = DBL_MAX;
        std::vector<bool> generated_vect = {false, false, false, false};
        std::vector<int> auto_mode_vect = {-1, -1, -1, -1};
        std::vector<double> delays_vect = {0, 0, 0, 0};
        int start_pos = 0;
        bool match_data_received = false;
        int auto_mode = -1;
        int layout = 0;
        bool in_auto = false;
        bool mode_buffered = false;
        bool end_auto = false;
        bool in_teleop = false;
        //ROS_WARN("Start of auto loop");
		ROS_ERROR("Here8");
        while(!in_teleop && !end_auto) {
            ////ROS_WARN("In auto loop");

            MatchData match_data = *(matchData.readFromRT());
            AutoMode auto_mode_data = *(autoMode.readFromRT());

            if(!in_auto) { //accept changes to chosen auto_modes until we receive match data or auto starts
                ////ROS_INFO("No match data and not in auto");
                //loop through auto_mode data 
                //generate trajectories for all changed modes
                for(int i = 0; i<4; i++) {
                    if ((auto_mode_data.modes_[i] > num_cmd_vel_modes-1) &&
                        ((auto_mode_data.modes_[i] != auto_mode_vect[i]) || (auto_mode_data.start_pos_ != start_pos)))
                    {
                        if(generateTrajectory(profiled_modes[auto_mode_data.modes_[i] - num_cmd_vel_modes][i][auto_mode_data.start_pos_])) {
                            generated_vect[i] = true;
                            auto_mode_vect[i] = auto_mode_data.modes_[i];
                            delays_vect[i] = auto_mode_data.delays_[i];
                            start_pos = auto_mode_data.start_pos_;
                            //ROS_WARN("Generating Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
                        }
                        else {
                            auto_mode_vect[i] = -1;
                            start_pos = -1;
                            generated_vect[i] = false;
                            //ROS_WARN("Invalid Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
                        }
                    }
                    else if (auto_mode_data.modes_[i] <= num_cmd_vel_modes-1 && (auto_mode_data.modes_[i] >= 0) &&
                        ((auto_mode_data.modes_[i] != auto_mode_vect[i]) || (auto_mode_data.start_pos_ != start_pos)))
                    {
                        if(generateTrajectory(cmd_vel_modes[auto_mode_data.modes_[i]][i][auto_mode_data.start_pos_])) {
                            auto_mode_vect[i] = auto_mode_data.modes_[i];
                            delays_vect[i] = auto_mode_data.delays_[i];
                            start_pos = auto_mode_data.start_pos_;
                            generated_vect[i] = true;
                            //ROS_WARN("Generating Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
                        }
                        else {
                            auto_mode_vect[i] = -1;
                            start_pos = -1;
                            generated_vect[i] = false;
                            //ROS_WARN("Invalid Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
                        }
                    }
                }
            }

            if(in_auto && match_data.isAutonomous_ == false) {
                //ROS_WARN("Leaving Autonomous to teleop");
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
            if(!match_data_received && ros::Time::now().toSec() > auto_start_time + wait_for_match_data) { //if match data isn't found after 2 seconds of auto starting run default auto
                //ROS_INFO("In first two seconds of auto with no match data");
                layout = 0;
                auto_mode_vect[layout] = 1; //default auto: cross baseline forward
                generated_vect[layout] = true;
                match_data_received = true;
                mode_buffered = true;
            }
            if(!in_auto) { //check for auto to start and set a start time
                ////ROS_INFO("Not in auto yet");
                if(match_data.isAutonomous_ && match_data.isEnabled_) {
                    //ROS_WARN("Entering Auto");
                    in_auto = true;
                    auto_start_time = ros::Time::now().toSec();
                }
            }
            if(in_auto) {
                //ROS_INFO("In auto");
                if(!match_data.isAutonomous_ || !match_data.isEnabled_) {
                    //ROS_WARN("Disabled in Auto");
                    in_auto = false;
                    auto_start_time = DBL_MAX;
                }
            }

            if(match_data_received && !mode_buffered) { //if we have match data and haven't buffered yet, buffer
                //ROS_INFO("Match data received no auto buffered yet");
                if(generated_vect[layout]) {
                    if(auto_mode_vect[layout] > num_cmd_vel_modes-1) {
                        if(bufferTrajectory(profiled_modes[auto_mode_vect[layout]-num_cmd_vel_modes][layout][start_pos].srv_msg.response)) {
                            //ROS_WARN("Buffering Profiled auto mode");
                            mode_buffered = true;
                        }
                    }
                    else if(auto_mode_vect[layout] > 0) {
                        //ROS_WARN("Fake buffering cmd_vel auto mode");
                        mode_buffered = true;
                    }
                }
                else {
                    //ROS_WARN("No path generated when match_data received, Layout: [%d]", layout);
                }
            }

            if(in_auto && mode_buffered) { //if in auto with mode buffered run it
                //ROS_INFO("Match data received and auto buffered");
                if(auto_mode_vect[layout] > num_cmd_vel_modes-1) {
                    run_auto(auto_mode_vect[layout], layout, start_pos, 
                             delays_vect[layout], profiled_modes[auto_mode_vect[layout]-num_cmd_vel_modes][layout][start_pos].actions);
                }
                else if(auto_mode_vect[layout] >= 0) {
                    run_auto(auto_mode_vect[layout], layout, start_pos, 
                             delays_vect[layout], cmd_vel_modes[auto_mode_vect[layout]][layout][start_pos].segments);
                    
                }
                //ROS_WARN("Running Auto");
                // Auto finished, either by finishing the requested path
                // or by match data reporting not in autonomous
                in_auto = false;
                end_auto = true;
            }

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
                in_teleop = !match_data.isAutonomous_ && !match_data.isEnabled_;
            }
			slow.sleep(); //I think you want this.....
        }
        while(in_teleop) {
            //ROS_INFO("Exited auto into teleop");
            MatchData match_data = *(matchData.readFromRT());
            if(!match_data.isEnabled_ || match_data.isAutonomous_) {
                in_teleop = false;
            }
            slow.sleep(); 
        }
    }
    return 0;
}
