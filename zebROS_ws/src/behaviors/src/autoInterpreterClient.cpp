#include <atomic>

#include <behaviors/autoInterpreterClient.h>
#include <realtime_tools/realtime_buffer.h>

std::atomic<bool> exit_auto; // robot is disabled or teleop started during auto_loop

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
bool intake_ready_to_drop_up_or_down;
double default_x;
double default_y;

struct MatchData {
    MatchData():
        isEnabled_(false),
        isAutonomous_(false),
        alliance_data_("")
    {
    }
    
    MatchData(bool isEnabled, bool isAutonomous, std::string alliance_data):
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
    
    AutoMode(std::vector<int> modes, std::vector<double> delays, int start_pos):
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
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in intakeConfig");
		return false;
	}
	return true;
}
bool switchConfig(void) {
	elevator_controller::ElevatorControlS srv;
    srv.request.x = switch_config_x;
    srv.request.y = switch_config_y;
    srv.request.up_or_down = true;
    srv.request.override_pos_limits = false;
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
    if (!ElevatorService.call(srv))
	{
		ROS_ERROR("Service call failed : ElevatorService in lowScale");
		return false;
	}
	return true;
}
bool stopIntake(void) {
	elevator_controller::Intake srv;
    srv.request.spring_state = 2; //soft_in
    srv.request.power=0;
    if (!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in stopIntake");
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

mode_list load_all_trajectories(int max_mode_num, int max_start_pos_num, ros::NodeHandle &auto_data)
{
	XmlRpc::XmlRpcValue mode_xml;
	mode_list all_modes;
	all_modes.resize(max_mode_num + 1);
	for(int mode = 0; mode <= max_mode_num; mode++)
	{
		all_modes[mode].resize(4);
		for(int layout = 0; layout <= 3; layout++)
		{
			all_modes[mode][layout].resize(max_start_pos_num + 1);
			for(int start_pos = 0; start_pos <= max_start_pos_num; start_pos++)
			{
				
				std::string identifier("mode_"+std::to_string(mode)+"_layout_"+
				std::to_string(layout)+"_start_"+std::to_string(start_pos));

				if(auto_data.getParam(identifier, mode_xml))
				{
					//ROS_INFO_STREAM("Auto mode with identifier: " << identifier << " found");
					const int num_splines = mode_xml.size();
					for(int num = 0; num<num_splines; num++) {
						XmlRpc::XmlRpcValue &spline = mode_xml[num];
						XmlRpc::XmlRpcValue &x = spline["x"];
						XmlRpc::XmlRpcValue &y = spline["y"];
						XmlRpc::XmlRpcValue &orient = spline["orient"];
						XmlRpc::XmlRpcValue &time = spline["time"];

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
						const double t = time;
						all_modes[mode][layout][start_pos].times.push_back(t);
						all_modes[mode][layout][start_pos].srv_msg.request.x_coefs.push_back(x_coefs);
						all_modes[mode][layout][start_pos].srv_msg.request.y_coefs.push_back(y_coefs);
						all_modes[mode][layout][start_pos].srv_msg.request.orient_coefs.push_back(orient_coefs);
						all_modes[mode][layout][start_pos].srv_msg.request.end_points.push_back(num+1);
					}
					all_modes[mode][layout][start_pos].srv_msg.request.initial_v = 0; 
					all_modes[mode][layout][start_pos].srv_msg.request.final_v = 0; 
					all_modes[mode][layout][start_pos].exists = true; 

					XmlRpc::XmlRpcValue group_xml;
					if(auto_data.getParam(identifier + "_spline_group", group_xml))
					{
						//ROS_INFO_STREAM("Custom grouping for identifier: " << identifier << " found");
						for(int i = 0; i < group_xml.size(); i++)
						{	
							all_modes[mode][layout][start_pos].srv_msg.request.spline_groups.push_back(group_xml[i]);
						}
						XmlRpc::XmlRpcValue wait_xml;
						if(auto_data.getParam(identifier + "_waits", wait_xml))
						{
							//ROS_INFO_STREAM("Custom waits for identifier: " << identifier << " found");
							for(int i = 0; i < group_xml.size(); i++)
							{
								all_modes[mode][layout][start_pos].srv_msg.request.wait_before_group.push_back(wait_xml[i]);
							}
						}
						else
						{
							for(int i = 0; i < group_xml.size(); i++)
							{
								all_modes[mode][layout][start_pos].srv_msg.request.wait_before_group.push_back(.16);
							}
						}
						XmlRpc::XmlRpcValue shift_xml;
						if(auto_data.getParam(identifier + "_t_shifts", shift_xml))
						{
							ROS_INFO_STREAM("Custom shifts for identifier: " << identifier << " found");
							for(int i = 0; i < group_xml.size(); i++)
							{
								all_modes[mode][layout][start_pos].srv_msg.request.t_shift.push_back(shift_xml[i]);
							}
						}
						else
						{
							for(int i = 0; i < group_xml.size(); i++)
							{
								all_modes[mode][layout][start_pos].srv_msg.request.t_shift.push_back(0);
							}
						}
						XmlRpc::XmlRpcValue flip_xml;
						if(auto_data.getParam(identifier + "_flips", flip_xml))
						{
							ROS_INFO_STREAM("Custom flips for identifier: " << identifier << " found");
							for(int i = 0; i < group_xml.size(); i++)
							{
								all_modes[mode][layout][start_pos].srv_msg.request.flip.push_back(flip_xml[i]);
							}
						}
						else
						{
							for(int i = 0; i < group_xml.size(); i++)
							{
								all_modes[mode][layout][start_pos].srv_msg.request.flip.push_back(false);
							}
						}
					}
					else
					{
							all_modes[mode][layout][start_pos].srv_msg.request.flip.push_back(false);
							all_modes[mode][layout][start_pos].srv_msg.request.spline_groups.push_back(num_splines);
							all_modes[mode][layout][start_pos].srv_msg.request.wait_before_group.push_back(.16);
							all_modes[mode][layout][start_pos].srv_msg.request.t_shift.push_back(0);
					}
				}
			}

		}

	}
	return all_modes;
}

bool generateTrajectory(full_mode &trajectory) 
{
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
	///JUST FOR TESTING_______
	if (!bufferTrajectory(trajectory.srv_msg.response))
	{
		ROS_ERROR("bufferTrajectory call failed in autoInterpreterClient generateTrajectory()");
		return false;
	}
	///JUST FOR TESTING_______
	return true;
}

std::string lower(const std::string &str) {
    std::string new_string;
    for(int i = 0; i<str.size(); i++) {
       new_string += tolower(str[i]); 
    }
    return new_string;
}

bool bufferTrajectory(const swerve_point_generator::FullGenCoefs::Response &traj)
{
    //ROS_INFO_STREAM("Buffer trajectory num_points: " << traj.points.size() << " dt: " << traj.dt);
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.points = traj.points;
    swerve_control_srv.request.dt = traj.dt;
    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.clear  = true;
    swerve_control_srv.request.run    = false;
    swerve_control_srv.request.mode   = true;
    
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
    swerve_control_srv.request.clear  = false;
    swerve_control_srv.request.run    = true;
    swerve_control_srv.request.mode   = true;
    
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


std::shared_ptr<actionlib::SimpleActionClient<behaviors::RobotAction>> ac;
void run_auto(int auto_select, int auto_mode, int layout, int start_pos, double initial_delay, const std::vector<double> &times) {
    //ROS_WARN("auto entered");
    exit_auto = false;
    ros::Rate r(10);
    double start_time = ros::Time::now().toSec();
    
    while(!exit_auto && ros::Time::now().toSec() < start_time + initial_delay) {
        r.sleep(); 
    }
    start_time = ros::Time::now().toSec();

    /*-------------------- Basic Cross line cmd vel auto ---------------------*/
    if(auto_select == 1) {
        //ROS_WARN("Basic drive forward auto");
        geometry_msgs::Twist vel;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        switchConfig(); 

        /*---------------------- Our Switch is on the Right ------------------------*/

        if(auto_mode == 0 || auto_mode == 2) {

           /* Starting on the left -> go to the left */
           if(start_pos == 0) {
               const double delay = 6; //Going 3/4 way towards scale
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.05; //positive x a lot
                    vel.linear.y = 0.1; //positive y a little bit
                    VelPub.publish(vel);
                    //ROS_INFO("Here");
                    r.sleep();
                }
           }
           
           /* Starting on the right -> go to the right */
           if(start_pos == 2) {
               const double delay = 6; //Going 3/4 way towards scale
               while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.05; //positive x a lot
                    vel.linear.y = -0.1; //negative y a little bit
                    VelPub.publish(vel);
                    //ROS_INFO("Here");
                    r.sleep();
                }
            }

            /* Starting in the middle -> go to the left */
            else {
                const double delay = 3.5; //Going exactly to the switch that is not ours
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 0.875; //positive x some
                    vel.linear.y = 0.5; //positive y some
                    VelPub.publish(vel);
                    //ROS_INFO("Here");
                    r.sleep();
                }
            }
        }

        /*------------------ Our Switch is on the Left -------------------------*/

        else if(auto_mode == 1 || auto_mode == 3) { //goal is on the left

            /* Starting on the left -> go to the left */
            if(start_pos == 0){
                const double delay = 6; 
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.05; //positive x a lot
                    vel.linear.y = 0.1; //positive y a little bit
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting on the right -> go to the right */
            if(start_pos == 2) {
                const double delay = 6; //TODO
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.05; //positive x a lot
                    vel.linear.y = -0.1; //negative y a little bit
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting in the middle -> go to the right */
            else {
                const double delay = 3.5; //TODO
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 0.875; //positive x some
                    vel.linear.y = -0.3; //negative y some
                    VelPub.publish(vel);
                    r.sleep();
                }
            }
        }
        parkingConfig();
    }

    /*--------------------- Basic Switch Cmd Vel -------------------------*/

	else if(auto_select == 2) {
        //ROS_WARN("Basic switch auto mode");
        //basic switch cmd_vel
        geometry_msgs::Twist vel;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        double start_time = ros::Time::now().toSec();
        switchConfig(); 

        /* Our switch is on the right */
        if(auto_mode == 0 || auto_mode == 2) {

            /* Starting on the left go to the right */ 
            if(start_pos == 0) {
               const double delay = 3; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.05;
                    vel.linear.y = -1.5;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting on the Right go to right switch */
            if(start_pos == 2) {
                const double delay = 2.58; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.2;
                    vel.linear.y = .53;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting in the middle go to right switch */
            else {
                const double delay = 3; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.05;
                    vel.linear.y = -0.45; 
                    VelPub.publish(vel);
                    r.sleep();
                }
				parkingConfig();
            }
            releaseClamp();
        }

        /* Our switch is on the left */
        else if(auto_mode == 1 || auto_mode == 3) {
            
            /* Starting on the left go to the left switch */
            if(start_pos == 0){
                const double delay = 2.58; //Going 1/2 meter past leading edge of our switch 
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.2;
                    vel.linear.y = -.53;
                    VelPub.publish(vel);
                    r.sleep();
                }
				parkingConfig();
            }

            /* Starting on the right go to the left switch */
            if(start_pos == 2) {
                const double delay = 3; //Going 1/2 meter past leading edge of our switch 
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.05;
                    vel.linear.y = -1.5;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting in the middle go to the left switch */
            else {
                const double delay = 2.76; //Going 1/2 meter past leading edge of our switch 
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = 1.125;
                    vel.linear.y = .675;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }
            releaseClamp();
        }
		parkingConfig();
    }
  /*-------------------------------Drive backwards and slam into switch--------------------*/
    else if(auto_select == 3)
    { 
        //ROS_WARN("Backwards switch auto mode");
        //basic switch cmd_vel
        geometry_msgs::Twist vel;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        double start_time = ros::Time::now().toSec();

        /* Our switch is on the right */
        if(auto_mode == 0 || auto_mode == 2) {

            /* Starting on the left go to the right */ 
            if(start_pos == 0) {
               const double delay = 3; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.05;
                    vel.linear.y = 1.5;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting on the Right go to right switch */
            if(start_pos == 2) {
                const double delay = 2.58; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.2;
                    vel.linear.y = -.53;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting in the middle go to right switch */
            else {
                const double delay = 3; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.05;
                    vel.linear.y = 0.45; 
                    VelPub.publish(vel);
                    r.sleep();
                }
            }
            parkingConfig();
        }

        /* Our switch is on the left */
        else if(auto_mode == 1 || auto_mode == 3) {

            /* Starting on the left go to the left switch */
            if(start_pos == 0){
                const double delay = 3; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.2;
                    vel.linear.y = .53;
                    VelPub.publish(vel);
                    r.sleep();
                }
				parkingConfig();
            }

            /* Starting on the right go to the left switch */
            if(start_pos == 2) {
                const double delay = 3.5; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.05;
                    vel.linear.y = -1.5;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting in the middle go to the left switch */
            else {
                const double delay = 3; //Goes 1/2 meter past leading edge of our switch
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.5;
                    vel.linear.y = -.75;
                    VelPub.publish(vel);
                    r.sleep();
                }
            }
        }
		parkingConfig();
    }


    /*-------------------- Backward Basic Cross line cmd vel auto ---------------------*/
    if(auto_select == 4) {
        //ROS_WARN("Basic drive forward auto");
        geometry_msgs::Twist vel;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        switchConfig(); 

        /*---------------------- Our Switch is on the Right ------------------------*/

        if(auto_mode == 0 || auto_mode == 2) {

           /* Starting on the left -> go to the left */
           if(start_pos == 0) {
               const double delay = 6; //Going 3/4 way towards scale
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.05; //positive x a lot
                    vel.linear.y = -0.1; //positive y a little bit
                    VelPub.publish(vel);
                    //ROS_INFO("Here");
                    r.sleep();
                }
           }
           
           /* Starting on the right -> go to the right */
           if(start_pos == 2) {
               const double delay = 6; //Going 3/4 way towards scale
               while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.05; //positive x a lot
                    vel.linear.y = 0.1; //negative y a little bit
                    VelPub.publish(vel);
                    //ROS_INFO("Here");
                    r.sleep();
                }
            }

            /* Starting in the middle -> go to the left */
            else {
                const double delay = 3.5; //Going exactly to the switch that is not ours
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -0.875; //positive x some
                    vel.linear.y = -0.5; //positive y some
                    VelPub.publish(vel);
                    //ROS_INFO("Here");
                    r.sleep();
                }
            }
        }

        /*------------------ Our Switch is on the Left -------------------------*/

        else if(auto_mode == 1 || auto_mode == 3) { //goal is on the left

            /* Starting on the left -> go to the left */
            if(start_pos == 0){
                const double delay = 6; 
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.05; //positive x a lot
                    vel.linear.y = -0.1; //positive y a little bit
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting on the right -> go to the right */
            if(start_pos == 2) {
                const double delay = 6; //TODO
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -1.05; //positive x a lot
                    vel.linear.y = 0.1; //negative y a little bit
                    VelPub.publish(vel);
                    r.sleep();
                }
            }

            /* Starting in the middle -> go to the right */
            else {
                const double delay = 3.5; //TODO
                while(ros::Time::now().toSec() < start_time + delay && !exit_auto) {
                    vel.linear.x = -0.875; //positive x some
                    vel.linear.y = 0.3; //negative y some
                    VelPub.publish(vel);
                    r.sleep();
                }
            }
        }
        parkingConfig();
    }

    /*--------------------------- Profiled Single Scale auto mode ------------------------*/

	else if(auto_select == 5) {
        //ROS_WARN("Profiled Scale");
        while (!exit_auto && !runTrajectory())
			r.sleep();
		double last_time = 0;
        while(!exit_auto) { 
            //Profiled scale
            const double curr_time = ros::Time::now().toSec();
            if(curr_time > times[0] && curr_time <= times[0] + (curr_time-last_time)) {
                //ROS_WARN("Profiled Scale elevator to mid reached");
                midScale();
            }
            if(curr_time > times[1] && curr_time <= times[1] + (curr_time-last_time)) {
                //ROS_WARN("Profiled Scale release clamp reached");
                releaseClamp();
                exit_auto = true;
            }
            last_time = curr_time;
            r.sleep();
        }
		parkingConfig();
    }
	else if(auto_select == 6) {
        //ROS_WARN("Profiled Scale");
        while (!exit_auto && !runTrajectory())
			r.sleep();
		double last_time = 0;
        while(!exit_auto) { 
            //Profiled scale
            const double curr_time = ros::Time::now().toSec();
            if(curr_time > times[0] && curr_time <= times[0] + (curr_time-last_time)) {
                //ROS_WARN("Profiled Scale elevator to mid reached");
                midScale();
            }
            if(curr_time > times[1] && curr_time <= times[1] + (curr_time-last_time)) {
                //ROS_WARN("Profiled Scale release clamp reached");
                releaseClamp();
            }
            if(curr_time > times[2] && curr_time <= times[2] + (curr_time-last_time)) {
                //ROS_WARN("Intaking Cube and going to intake config");
                //robot_goal.IntakeCube = true; 
            }
            last_time = curr_time;
            r.sleep();
        }
		parkingConfig();
    }
/*--------------------------Either 2 scale 1 switch OR 3 scale 1 switch, depending------------------------------*/
    else if(auto_select == 5) {
	    if((auto_mode == 3 && start_pos == 0) || (auto_mode == 2 && start_pos == 2)) //if we are on the same side as both the switch and scale
	    {
			//ROS_WARN("3 Scale 1 switch");	
			while(!exit_auto && !runTrajectory())
				r.sleep();
			double last_time = 0;
			while(!exit_auto) {
			//Profiled scale
				const double curr_time = ros::Time::now().toSec();
				/** SCALE 1 **/
				if(curr_time > times[0] && curr_time <= times[1] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale elevator to mid reached");
					midScale();
				}
				if(curr_time > times[1] && curr_time <= times[2] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale release clamp reached");
					releaseClamp();
				}
				if(curr_time > times[2] && curr_time <= times[3] + (curr_time-last_time)) {
					//ROS_WARN("Intaking Cube and going to intake config");
					//robot_goal.IntakeCube = true; 
					intakeConfig();
				}
				/** SCALE 2 **/
				if(curr_time > times[3] && curr_time <= times[4] + (curr_time-last_time)) { //TODO fix all times here
					//ROS_WARN("profiled scale elevator to mid reached");
					midScale();
				}
				if(curr_time > times[4] && curr_time <= times[5] + (curr_time-last_time)) {
					//ROS_WARN("profiled scale release clamp reached");
					releaseClamp();
				}
				if(curr_time > times[5] && curr_time <= times[6] + (curr_time-last_time)) {
					//ROS_WARN("intaking cube and going to intake config");
					//robot_goal.intakeCube = true;
					intakeConfig();
				}
				//scale 3
				if(curr_time > times[6] && curr_time <= times[7] + (curr_time-last_time)) { //TODO fix all times here
					//ROS_WARN("profiled scale elevator to mid reached");
					midScale();
				}
				if(curr_time > times[7] && curr_time <= times[8] + (curr_time-last_time)) {
					//ROS_WARN("profiled scale release clamp reached");
					releaseClamp();
				}
				if(curr_time > times[8] && curr_time <= times[9] + (curr_time-last_time)) {
					//ROS_WARN("intaking cube and going to intake config");
					//robot_goal.intakeCube = true; 
					intakeConfig();
				}
				/** SWITCH **/
				if(curr_time > times[9] && curr_time <= times[10] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale elevator to mid reached");
					switchConfig();
				}
				if(curr_time > times[10] && curr_time <= times[11] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale release clamp reached");
					releaseClamp();
					exit_auto = true;
				}
				last_time = curr_time;
				r.sleep();
			}
		}
	    else if	(start_pos != 1)
		{
			//ROS_WARN("2 Scale 1 switch");
			while (!exit_auto && !runTrajectory())
				r.sleep();
			double last_time = 0;
			while(!exit_auto)
			{
				//Profiled scale
				const double curr_time = ros::Time::now().toSec();
			    /** SCALE 1 **/
				if(curr_time > times[0] && curr_time <= times[0] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale elevator to mid reached");
					midScale();
				}
				if(curr_time > times[1] && curr_time <= times[1] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale release clamp reached");
					releaseClamp();
				}
				if(curr_time > times[2] && curr_time <= times[2] + (curr_time-last_time)) {
					//ROS_WARN("Intaking Cube and going to intake config");
					//robot_goal.IntakeCube = true;
				}
				/** SCALE 2 **/
				if(curr_time > times[0] && curr_time <= times[3] + (curr_time-last_time)) {
					//ROS_WARN("profiled scale elevator to mid reached");
					midScale();
				}
				if(curr_time > times[1] && curr_time <= times[4] + (curr_time-last_time)) {
					//ROS_WARN("profiled scale release clamp reached");
					releaseClamp();
				}
				if(curr_time > times[2] && curr_time <= times[5] + (curr_time-last_time)) {
					//ROS_WARN("intaking cube and going to intake config");
					//robot_goal.intakecube = true; 
				}
				/** SWITCH **/
				if(curr_time > times[0] && curr_time <= times[6] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale elevator to mid reached");
					switchConfig();
				}
				if(curr_time > times[1] && curr_time <= times[7] + (curr_time-last_time)) {
					//ROS_WARN("Profiled Scale release clamp reached");
					releaseClamp();
					exit_auto = true;
				}
				else ROS_INFO_STREAM("Do nothing, start_pos = 1");
				last_time = curr_time;
				r.sleep();
			}
		}
	    parkingConfig();
	}

	/***** 1 switch and 2 exchange *****/
	else if (auto_select == 6) {
        //ROS_WARN("Profiled Scale");
        while (!exit_auto && !runTrajectory())
			r.sleep();
		double last_time = 0;
        while (!exit_auto)
		{
		    /** SWITCH 1 **/
		    if (curr_time > times[0] && curr_time <= times[1] + (curr_time-last_time))
			{
				//ROS_WARN("Profiled Switch Config");
				switchConfig();
		    }
		    if (curr_time > times[1] && curr_time <= times[7] + (curr_time-last_time))
			{
				//ROS_WARN("Profiled Intake Config");
				releaseClamp();
				exit_auto = true;
		    }
			/** EXCHANGE 1 **/
			if (curr_time > times[0] && curr_time <= times[1] + (curr_time - last_time))
			{
				//ROS_WARN("Profiled Expel Cube");
				instakeConfig();
			}
			if (curr_time > times[0] && curr_time <= times[1] + (curr_time - last_time))
			{
				//ROS_WARN("Profiled Expel Cube");
				releaseIntake(); //TODO: Is this the right function?
			}
            last_time = curr_time;
            r.sleep();
        }
		parkingConfig();
    }


    /*
    if(AutoMode->mode[auto_mode]==1) {
    //3 cube switch-scale-scale
        //0: Time 1: Go to switch config
        ros::Duration(times[0]).sleep(); //TODO
        switchConfig();
        //1: Time 2: Release Clamp
        ros::Duration(times[1]).sleep(); //TODO
        releaseClamp();

        //2: Time 3: drop and start intake && go to intake config
        ros::Duration(times[2]).sleep(); //TODO
        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //3: Linebreak sensor: Clamp && release intake && stop running intake

        bool success = ac->waitForResult(ros::Duration(timeout)); //TODO

        if(!success) {
            //ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake();

        intakeConfig(); 
        ros::Duration(.2).sleep();

        clamp();

        releaseIntake(); 
        
        //4: Time 4: go to scale config && soft-in intake
        ros::Duration(times[3]).sleep();
        midScale(); 
        r.sleep() //TODO

        IntakeSrv.request.spring_state = 2; //soft in
        IndtakeService.call();

        //6: Time 5: release Clamp
        ros::Duration(times[4]).sleep(); //TODO
        releaseClamp();

        //7: Time 6: go to intake config && run intake
        ros::Duration(times[5]).sleep(); // TODO
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //8: Linebreak sensor: Clamp && release intake && stop running intake
        success = ac->waitForResult(ros::Duration(timeout));
        
        if(!success) {
            ROS_ERROR("Failed to intake cube: TIME OUT");
        }
        stopIntake();

        clamp();

        releaseIntake();
        //10: Time 7: go to mid scale config && soft in intake
        ros::Duration(times[6]).sleep(); //TODO
        midScale();
        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call();
        
        //11: Time 8: release Clamp
        ros::Duration(times[7]).sleep(); //TODO
        releaseClamp();

        //12: Time 9: go to intake config && start intake
        ros::Duration(times[8]).sleep(); //TODO
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);
        //13: Linebreak sensor: Clamp and release cube
        clamp();
        releaseIntake();
    }
    else if(AutoMode->mode[auto_mode-1]==2) {
        //2 cube long std_srvs::SetBool
        //0: Time 0: Go to default config && drop intake
        ros::Duration(0).sleep(); //TODO
        defaultConfig();

        IntakeSrv.request.up = false;
        IntakeSrv.request.spring_state = 2; //soft_in

        IntakeService.call();
                
        //1: Time 1: Go to mid scale config && Release Clamp
        ros::Duration(1).sleep(); //TODO
        midScale();
        ros::Duration(.3).sleep(); //TODO

        releaseClamp();

        //2: Time 1.5: go to intake config && start intake
        ros::Duration(1).sleep(); //TODO
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //3: Linebreak sensor: Clamp && release intake && stop running intake
        bool success = ac->waitForResult(ros::Duration(timeout)); //TODO
        stopIntake();

        if(!success) {
            ROS_ERROR("Failed to intake cube! TIME OUT");
        }

        clamp();

        releaseIntake();

        //4: Success of command 3: go to default config && soft-in intake
        defaultConfig(); 
        ros::Duration(.2).sleep(); //TODO
        IntakeSrv.request.spring_state = 2; //soft in
        IntakeService.call();
       
        //5: Time 2: go to mid level scale config
        ros::Duration(2).sleep(); //TODO
        midScale();

        //6: Time 2.5: release Clamp
        ros::Duration(2.5).sleep(); //TODO
        releaseClamp();

        //7: Time 3: go to intake config && run intake
        ros::Duration(3).sleep(); //TODO
        intakeConfig(); 

        goal.IntakeCube = true;
        ac->sendGoal(goal);
        
        success = ac->waitForResult(ros::Duration(timeout));

        //8: Linebreak sensor: Clamp && release intake && stop running intake
        stopIntake();
        if(!success) {
            //ROS_WARN("Failed to intake cube: TIME OUT");
        }

        clamp();
        releaseIntake();


    }
    else if(AutoMode->mode[auto_mode]==3) {
    //3 cube switch-scale-scale
        //0: Time 1: Go to mid scale config
        ros::Duration(times[0]).sleep(); //TODO
        midScale();
        //1: Time 2: Release Clamp
        ros::Duration(times[1]).sleep(); //TODO
        releaseClamp();

        //2: Time 3: drop and start intake && go to intake config
        ros::Duration(times[2]).sleep(); //TODO
        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //3: Linebreak sensor: Clamp && release intake && stop running intake

        bool success = ac->waitForResult(ros::Duration(timeout)); //TODO

        if(!success) {
            //ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake();

        intakeConfig(); 
        ros::Duration(.2).sleep();

        clamp();

        releaseIntake(); 
        
        //4: Time 4: go to switch config && soft-in intake
        ros::Duration(times[3]).sleep();
        switchConfig(); 
        r.sleep() //TODO

        IntakeSrv.request.spring_state = 2;
        IntakeService.call();

        //6: Time 5: release Clamp
        ros::Duration(times[4]).sleep(); //TODO
        releaseClamp();

        //7: Time 6: go to intake config && run intake
        ros::Duration(times[5]).sleep(); // TODO
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //8: Linebreak sensor: Clamp && release intake && stop running intake
        success = ac->waitForResult(ros::Duration(timeout));
        
        if(!success) {
            ROS_ERROR("Failed to intake cube: TIME OUT");
        }
        stopIntake();

        clamp();

        releaseIntake();
        //10: Time 7: go to mid scale config && soft in intake
        ros::Duration(times[6]).sleep(); //TODO
        midScale();
        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call();
        
        //11: Time 8: release Clamp
        ros::Duration(times[7]).sleep(); //TODO
        releaseClamp();

        //12: Time 9: go to intake config && start intake
        ros::Duration(times[8]).sleep(); //TODO
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);
        //13: Linebreak sensor: Clamp and release cube
        clamp();
        releaseIntake();
    
}
    else if(AutoMode->mode[auto_mode]==4) { //TODO fix times
    //4 cube scale-scale-scale-switch
        //1: Time 1: scale config and soft-in intake
        ros::Duration(times[0]).sleep(); //good
        midScale();

        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call();

        //2: Time 2: Release clamp
        ros::Duration(times[4]).sleep(); //good
        releaseClamp();

        //3: Time 3: intake config and run intake
        ros::Duration(times[5]).sleep();
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //4: Linebreak sensor: Clamp, release intake, stop running intake
        bool success = ac->waitForResult(ros::Duration(timeout));

        if(!success) {
            //ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake();

        intakeConfig(); 
        ros::Duration(.2).sleep();

        clamp();

        releaseIntake(); 

        //5: Time next: go to scale config and soft-in intake 
        ros::Duration(times[0]).sleep();
        midScale();

        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call();

        //6: Time 1: Release clamp
        ros::Duration(times[4]).sleep();
        releaseClamp();

        //7: Time 2: intake config and run intake
        ros::Duration(times[5]).sleep();
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //8: Linebreak sensor: clamp, release intake, stop running intake
        success = ac->waitForResult(ros::Duration(timeout));

        if(!success) {
            //ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake();

        intakeConfig(); 
        ros::Duration(.2).sleep();

        clamp();

        releaseIntake(); 

        //9: Time next: go to scale config and soft-in intake 
        ros::Duration(times[0]).sleep();
        midScale();

        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call();

        //10: Time 1: Release clamp
        ros::Duration(times[4]).sleep();
        releaseClamp();

        //11: Time 2: intake config and run intake
        ros::Duration(times[5]).sleep();
        intakeConfig();

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //12: Linebreak sensor: clamp, release intake, stop running intake
        success = ac->waitForResult(ros::Duration(timeout));

        if(!success) {
            //ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake();

        intakeConfig(); 
        ros::Duration(.2).sleep();

        clamp();

        releaseIntake();

        //13: Time something: switch config and soft-in intake
        ros::Duration(times[0]).sleep();
        switchConfig();

        IntakeSrv.request.spring_state=2; //spring state
        IntakeService.call();

        //14: Time something: release clamp
        ros::Duration(times[4]).sleep();
        releaseClamp();

    }
    else if(AutoMode->mode[auto_mode]==5) {
    //1 cube simple switch
        //1: Time 1: Go to switch config and soft_in intake
        ros::Duration(times[0]).sleep();
        switchConfig();

        ros::Duration(times[1]).sleep();
        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call();

        //2: Time 2: Release clamp
        ros::Duration(times[2]).sleep();
        releaseClamp();
    }
    else if(AutoMode->mode[auto_mode]==6) {

    }
    else if(AutoMode->mode[auto_mode]==7) {

    }
    else if(AutoMode->mode[auto_mode]==8) {

    }
    else if(AutoMode->mode[auto_mode]==9) {

    }
    else if(AutoMode->mode[auto_mode]==10) {

    }
    else if(AutoMode->mode[auto_mode]==11) {

    }
    else if(AutoMode->mode[auto_mode]==12) {

    }
    else{
        
    }
*/
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Auto_state_subscriber");
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");

    ros::NodeHandle n_params_behaviors(n, "auto_params");
   
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
    if (!n_params.getParam("default_x", default_x))
		ROS_ERROR("Didn't read param default_x in autoInterpreterClient");
    if (!n_params.getParam("default_y", default_y))
		ROS_ERROR("Didn't read param default_y in autoInterpreterClient");

    int max_num_mode;
    int max_num_start;

    if (!n_params_behaviors.getParam("max_num_mode", max_num_mode))
		ROS_ERROR("Didn't read param max_num_mode in autoInterpreterClient");
    if (!n_params_behaviors.getParam("max_num_start", max_num_start))
		ROS_ERROR("Didn't read param max_num_start in autoInterpreterClient");

	// Load trajectories before callbacks which use them can
	// start
	mode_list all_modes = load_all_trajectories(max_num_mode, max_num_start, n_params_behaviors);

    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::RobotAction>>("auto_interpreter_server", true);
    ac->waitForServer(); 

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	point_gen = n.serviceClient<swerve_point_generator::FullGenCoefs>("/point_gen/command", true, service_connection_header);
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);

    IntakeService = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
    ElevatorService = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
    ClampService = n.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
    BrakeService = n.serviceClient<std_srvs::Empty>("/frcrobot/talon_swerve_drive_controller/brake", false, service_connection_header);
    
    VelPub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);

    ros::Subscriber auto_mode_sub = n.subscribe("autonomous_mode", 1, &auto_mode_cb);
    ros::Subscriber match_data_sub = n.subscribe("match_data", 1, &match_data_cb);

	// Kick off 2 threads to process messages
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
    ROS_WARN("Auto Client loaded");
    //ros::Duration(20).sleep();
    ROS_WARN("post sleep");
    
    /*---------------------------- JUST FOR TESTING ------------------------------------ */
    //generateTrajectory(all_modes[4][3][2]);
    //ROS_WARN("Auto Client loaded");
    //ros::Duration(30).sleep();
    //ROS_WARN("post sleep");
    
    /*---------------------------- JUST FOR TESTING ------------------------------------ */
    //generateTrajectory(all_modes[3][3][2]);
    /*---------------------------- JUST FOR TESTING ------------------------------------ */

    //ROS_WARN("SUCCESS IN autoInterpreterClient.cpp");
    ros::Rate r(10);

    while(ros::ok()) {
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
        while(!in_teleop && !end_auto) {
            ////ROS_WARN("In auto loop");

            MatchData match_data = *(matchData.readFromRT());
            AutoMode auto_mode_data = *(autoMode.readFromRT());

            if(!match_data_received && !in_auto) { //accept changes to chosen auto_modes until we receive match data or auto starts
                ////ROS_INFO("No match data and not in auto");
                //loop through auto_mode data 
                //generate trajectories for all changed modes
                for(int i = 0; i<4; i++) {
                    if ((auto_mode_data.modes_[i] > 4) &&
                        ((auto_mode_data.modes_[i] != auto_mode_vect[i]) || (auto_mode_data.start_pos_ != start_pos)))
                    {
                        if(generateTrajectory(all_modes[i][auto_mode_data.modes_[i] - 5][auto_mode_data.start_pos_])) {
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
                    else if (auto_mode_data.modes_[i] <= 4 && (auto_mode_data.modes_[i] >= 0) &&
                        ((auto_mode_data.modes_[i] != auto_mode_vect[i]) || (auto_mode_data.start_pos_ != start_pos)))
                    {
                        auto_mode_vect[i] = auto_mode_data.modes_[i];
                        delays_vect[i] = auto_mode_data.delays_[i];
                        start_pos = auto_mode_data.start_pos_;
                        generated_vect[i] = true;
                        //ROS_WARN("Generating Auto mode [%d], to be mode: %d", i, auto_mode_data.modes_[i]);
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
                    auto_mode = 0;
                    layout = 1;
                }
                else if(lower(match_data.alliance_data_)=="lrl") {
                    auto_mode = 1;
                    layout = 1;
                }
                else if(lower(match_data.alliance_data_)=="rrr") {
                    auto_mode = 2;
                    layout = 2;
                }
                else if(lower(match_data.alliance_data_) =="lll") {
                    auto_mode = 3;
                    layout = 2;
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
            if(!match_data_received && ros::Time::now().toSec() > auto_start_time + 2) { //if match data isn't found after 2 seconds of auto starting run default auto
                //ROS_INFO("In first two seconds of auto with no match data");
                auto_mode = 0;
                auto_mode_vect[0] = 1; //default auto: cross baseline
                generated_vect[0] = true;
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
                if(generated_vect[auto_mode]) {
                    if(auto_mode_vect[auto_mode] > 4) {
                        if(bufferTrajectory(all_modes[auto_mode_vect[auto_mode]][auto_mode][start_pos].srv_msg.response)) {
                            //ROS_WARN("Buffering Profiled auto mode");
                            mode_buffered = true;
                        }
                    }
                    else {
                        //ROS_WARN("Fake buffering cmd_vel auto mode");
                        mode_buffered = true;
                    }
                }
                else {
                    //ROS_WARN("No path generated when match_data received, Auto Mode: [%d]", auto_mode);
                }
            }

            if(in_auto && mode_buffered) { //if in auto with a mode buffered run it
                //ROS_INFO("Match data received and auto buffered");
                run_auto(auto_mode_vect[auto_mode], auto_mode, layout, start_pos, 
                         delays_vect[auto_mode], all_modes[auto_mode][layout][start_pos].times);
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
                in_teleop = !match_data.isAutonomous_;
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
