#include <atomic>
#include <mutex>

#include <behaviors/autoInterpreterClient.h>

ros::ServiceClient point_gen;
ros::ServiceClient swerve_control;

// Stuff shared between callbacks and main code
// Wrap these in 1 realtime buffer per callback
// writefromnonRt in callback, readFromRT in
// run_auto and other uses
static std::atomic<int> start_pos; // or maybe just bundle with mutex below
std::vector<int> auto_mode_vect = {0, 0, 0, 0};
std::vector<double> delays_vect = {0, 0, 0, 0};
std::mutex auto_mode_delays_mutex;
static std::atomic<int> auto_mode;
static std::atomic<int> layout;
static std::atomic<bool> in_auto;

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
double intake_config_x;
double intake_config_y;
double default_x;
double default_y;
double timeout;
double delay;
double autoStart = -1;
XmlRpc::XmlRpcValue modes;
std::vector<trajectory_msgs::JointTrajectory> trajectories;


//consider adding some constructors for these structs

struct full_mode
{
	talon_swerve_drive_controller::FullGenCoefs srv_msg;
	std::vector<double> times;
	bool exists;

	full_mode(void): exists(false) {}	
};

typedef std::vector<std::vector<std::vector<full_mode>>> mode_list;

static mode_list all_modes;

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
    srv.request.x = intake_config_x;
    srv.request.y = intake_config_y;
    srv.request.up_or_down = false;
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
    srv.request.spring_state = 1;
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
	ROS_WARN("Braking");
	if (!BrakeService.call(empty))
	{
		ROS_ERROR("Service call failed : BrakeService in parkingConfig");
		return false;
	}
	return true;
}

void load_all_trajectories(int max_mode_num, int max_start_pos_num, ros::NodeHandle &auto_data)
{
	XmlRpc::XmlRpcValue mode_xml;
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
					ROS_INFO_STREAM("Auto mode with identifier: " << identifier << " found");
					const int num_splines = mode_xml.size();
					for(int num = 0; num<num_splines; num++) {
						XmlRpc::XmlRpcValue &spline = mode_xml[num];
						XmlRpc::XmlRpcValue &x = spline["x"];
						XmlRpc::XmlRpcValue &y = spline["y"];
						XmlRpc::XmlRpcValue &orient = spline["orient"];
						XmlRpc::XmlRpcValue &time = spline["time"];

						talon_swerve_drive_controller::Coefs x_coefs;
						talon_swerve_drive_controller::Coefs y_coefs;
						talon_swerve_drive_controller::Coefs orient_coefs;
						for(int i = 0; i<x.size(); i++) {
						    const double x_coef = x[i];
						    const double y_coef = y[i];
						    const double orient_coef = orient[i];

						    //ROS_WARN("%f", orient_coef);
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
						ROS_INFO_STREAM("Custom grouping for identifier: " << identifier << " found");
						for(int i = 0; i < group_xml.size(); i++)
						{	
							all_modes[mode][layout][start_pos].srv_msg.request.spline_groups.push_back(group_xml[i]);
						}
						XmlRpc::XmlRpcValue wait_xml;
						if(auto_data.getParam(identifier + "_waits", wait_xml))
						{
							ROS_INFO_STREAM("Custom waits for identifier: " << identifier << " found");
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
					}
					else
					{
							all_modes[mode][layout][start_pos].srv_msg.request.spline_groups.push_back(num_splines);
							all_modes[mode][layout][start_pos].srv_msg.request.wait_before_group.push_back(.16);
					}
				}
			}

		}

	}
}

bool generateTrajectory(int auto_mode, int layout, int start_pos) {
    
    if(!all_modes[auto_mode][layout][start_pos].exists)
    {
	//TODO MAKE LIGHT GO RED ON DRIVERSTATION
	ROS_ERROR("auto mode/layout/start selected which wasn't found in the yaml");
    	return false;
    }
    
    if (!point_gen.call(all_modes[auto_mode][layout][start_pos].srv_msg))
    {
		ROS_ERROR("point_gen call failed in autoInterpreterClient generateTrajectory()");
    	return false;
    }
    ///JUST FOR TESTING_______
    bufferTrajectory(auto_mode, layout, start_pos);
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

void bufferTrajectory(int auto_mode, int layout, int start_pos) {
    ROS_WARN("Buffer trajectory");
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.points = all_modes[auto_mode][layout][start_pos].srv_msg.response.points;
    ROS_INFO_STREAM("num_points: " << all_modes[auto_mode][layout][start_pos].srv_msg.response.points.size() << " dt: "<<all_modes[auto_mode][layout][start_pos].srv_msg.response.dt);
    
    swerve_control_srv.request.dt = all_modes[auto_mode][layout][start_pos].srv_msg.response.dt;
    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.clear  = true;
    swerve_control_srv.request.run    = false;
    swerve_control_srv.request.mode   = true;
    
    if (!swerve_control.call(swerve_control_srv))
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient bufferTrajectory()");
}
void runTrajectory(int auto_mode) {
    ROS_WARN("Run trajectory");
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.buffer = false;
    swerve_control_srv.request.clear  = false;
    swerve_control_srv.request.run    = true;
    swerve_control_srv.request.mode   = true;
    
    if (!swerve_control.call(swerve_control_srv))
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient runTrajectory()");
}

void auto_mode_cb(const ros_control_boilerplate::AutoMode::ConstPtr &AutoMode) {
	for(int i = 0; i<4; i++) {
		if ((AutoMode->mode[i] > 2) &&
			((AutoMode->mode[i] != auto_mode_vect[i]) || (AutoMode->position != start_pos)))
		{
			generateTrajectory(i, AutoMode->mode[i]-3, AutoMode->position);
		}
		{
			std::lock_guard<std::mutex> l(auto_mode_delays_mutex);
			auto_mode_vect[i] = AutoMode->mode[i];
			delays_vect[i] = AutoMode->delays[i];
		}
	}
	start_pos = AutoMode->position;
}

void match_data_cb(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {
    if(MatchData->isEnabled && MatchData->isAutonomous && MatchData->allianceData != "") {
        if(lower(MatchData->allianceData)=="rlr") {
            auto_mode = 0;
            layout = 1;
        }
        else if(lower(MatchData->allianceData)=="lrl") {
    //ROS_WARN("auto entered");
        //ROS_WARN("2");
            auto_mode = 1;
            layout = 1;
        }
        else if(lower(MatchData->allianceData)=="rrr") {
            auto_mode = 2;
            layout = 2;
        }
        else if(lower(MatchData->allianceData) =="lll") {
    //ROS_WARN("auto entered");
        //ROS_WARN("3");
            auto_mode = 3;
            layout = 2;
        }
        in_auto = true;
        run_auto(auto_mode);
    }
    else if (MatchData->allianceData != "") {
        if(lower(MatchData->allianceData)=="rlr") {
            auto_mode = 0;
            layout = 1;
        }
        else if(lower(MatchData->allianceData)=="lrl") {
    //ROS_WARN("auto entered");
        //ROS_WARN("2");
            auto_mode = 1;
            layout = 1;
        }
        else if(lower(MatchData->allianceData)=="rrr") {
            auto_mode = 2;
            layout = 2;
        }
        else if(lower(MatchData->allianceData) =="lll") {
    //ROS_WARN("auto entered");
        //ROS_WARN("3");
            auto_mode = 3;
            layout = 2;
        }
		// TODO :: Check this - are we really in auto mode if in this else() block
        in_auto = true;
        
        if(auto_mode_vect[auto_mode] > 2) {
            //bufferTrajectory(auto_mode);
	    //TODO WHAT IS THIS FOR???
        }
    }
    else {
        in_auto = false;
    }
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::RobotAction>> ac;
void run_auto(int auto_mode) {
    ROS_WARN("auto entered");
    ros::Rate r(10);
    double start_time = ros::Time::now().toSec();
	double initial_delay;
	{
		std::lock_guard<std::mutex> l(auto_mode_delays_mutex);
		initial_delay = delays_vect[auto_mode];
	}

    while(in_auto && ros::Time::now().toSec() < start_time + initial_delay) {
        r.sleep(); 
        ros::spinOnce();
    }
    start_time = ros::Time::now().toSec();

	// Lock in auto mode info here in case it somehow changes
	// in callback?  Running based on data which is changing
	// could be dangerous
	int auto_mode_vect_auto_mode;
	{
		std::lock_guard<std::mutex> l(auto_mode_delays_mutex);
		auto_mode_vect_auto_mode = auto_mode_vect[auto_mode];
	}
	const int this_start_pos = start_pos;

    if(auto_mode_vect_auto_mode == 1) {
        while(in_auto) {
            ROS_WARN("Basic drive forward auto");
            //basic drive forward cmd_vel
            geometry_msgs::Twist vel;
            vel.linear.z = 0;
            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = 0;

            switchConfig(); 
            if(auto_mode == 1 || auto_mode == 3) { //if goal is on the right
               if(this_start_pos == 0) {
                   const double delay = 6; //TODO
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = 0.12; //positive y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
               }
               if(this_start_pos == 2) {
                   const double delay = 6; //TODO
                   while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = -0.12; //negative y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
                else {
                    const double delay = 3.04; //TODO
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 0.875; //positive x some
                        vel.linear.y = 0.5; //positive y some
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
            }
            else if(auto_mode == 2 || auto_mode == 4) { //goal is on the left
                if(this_start_pos == 0){
                    const double delay = 6; 
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = 0.12; //positive y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
                if(this_start_pos == 2) {
                    const double delay = 6; //TODO
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = -0.12; //negative y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
                else {
                    const double delay = 3.04; //TODO
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 0.875; //positive x some
                        vel.linear.y = -0.3; //negative y some
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
            }
			// TODO : brake if in_auto is set false by match data?
			parkingConfig();
            in_auto = false;
        }
    }
	else if(auto_mode_vect_auto_mode == 2) {
        ROS_WARN("Basic switch auto mode");
        //basic switch cmd_vel
        geometry_msgs::Twist vel;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        double start_time = ros::Time::now().toSec();
        switchConfig(); 
        if(auto_mode == 1 || auto_mode == 3) { //if goal is on the right
           if(this_start_pos == 0) {
               const double delay = 3.5; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.05;
                    vel.linear.y = -1.5;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
            }
           if(this_start_pos == 2) {
            const double delay = 3; //TODO
               while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.2;
                    vel.linear.y = .53;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
           }
            else {
                const double delay = 3; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.75;
                    vel.linear.y = -0.7; 
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
				parkingConfig();
            }
            releaseClamp();
        }
        else if(auto_mode == 2 || auto_mode == 4) { //goal is on the left
            if(this_start_pos == 0){
                const double delay = 3; 
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.2;
                    vel.linear.y = -.53;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
				parkingConfig();
            }
            if(this_start_pos == 2) {
                const double delay = 3.5; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.05;
                    vel.linear.y = 1.5;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
            }
            else {
                const double delay = 3; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.5;
                    vel.linear.y = .75;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
            }
            releaseClamp();
        }
		parkingConfig();
        in_auto = false;
    }
    
	else if(auto_mode_vect_auto_mode == 3) {
        ROS_WARN("Profiled Scale");
        runTrajectory(auto_mode);
		double last_time = 0;
		const auto &times = all_modes[auto_mode][layout][start_pos].times;
        while(in_auto) { 
            //Profiled scale
            const double curr_time = ros::Time::now().toSec();
            if(curr_time > times[0] && curr_time <= times[0] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale elevator to mid reached");
                midScale();
            }
            if(curr_time > times[1] && curr_time <= times[1] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale release clamp reached");
                releaseClamp();
            }
            last_time = curr_time;
            r.sleep();
            ros::spinOnce();
        }
		parkingConfig();
        in_auto = false;
    }
	else if(auto_mode_vect_auto_mode == 4) {
        ROS_WARN("Profiled Scale");
        runTrajectory(auto_mode);
		double last_time = 0;
		const auto &times = all_modes[auto_mode][layout][start_pos].times;
        while(in_auto) { 
            //Profiled scale
            const double curr_time = ros::Time::now().toSec();
            if(curr_time > times[0] && curr_time <= times[0] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale elevator to mid reached");
                midScale();
            }
            if(curr_time > times[1] && curr_time <= times[1] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale release clamp reached");
                releaseClamp();
            }
            if(curr_time > times[2] && curr_time <= times[2] + (curr_time-last_time)) {
                ROS_WARN("Intaking Cube and going to intake config");
                //robot_goal.IntakeCube = true; 
            }
            last_time = curr_time;
            r.sleep();
            ros::spinOnce();
        }
		parkingConfig();
        in_auto = false;
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
            ROS_WARN("Failed to intake cube: TIME OUT");
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

        IntakeSrv.request.spring_state = 2;
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
            ROS_WARN("Failed to intake cube: TIME OUT");
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
            ROS_WARN("Failed to intake cube: TIME OUT");
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
            ROS_WARN("Failed to intake cube: TIME OUT");
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
            ROS_WARN("Failed to intake cube: TIME OUT");
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
            ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake();

        intakeConfig(); 
        ros::Duration(.2).sleep();

        clamp();

        releaseIntake();

        //13: Time something: switch config and soft-in intake
        ros::Duration(times[0]).sleep();
        switchConfig();

        IntakeSrv.request.spring_state=2;
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
    if (!n_params.getParam("intake_config_x", intake_config_x))
		ROS_ERROR("Didn't read param intake_config_x in autoInterpreterClient");
    if (!n_params.getParam("intake_config_y", intake_config_y))
		ROS_ERROR("Didn't read param intake_config_y in autoInterpreterClient");
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

	start_pos = 0;
	auto_mode = -1;
	layout = 0;
	in_auto = false;

	// Load trajectories before callbacks which use them can
	// start
    load_all_trajectories(max_num_mode, max_num_start, n_params_behaviors);

    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::RobotAction>>("auto_interpreter_server", true);
    ac->waitForServer(); 

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	point_gen = n.serviceClient<talon_swerve_drive_controller::FullGenCoefs>("/point_gen/command", true, service_connection_header);
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);

    IntakeService = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
    ElevatorService = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
    ClampService = n.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
    BrakeService = n.serviceClient<std_srvs::Empty>("/frcrobot/talon_swerve_drive_controller/brake", false, service_connection_header);
    
    VelPub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);

    ros::Subscriber auto_mode_sub = n.subscribe("autonomous_mode", 1, &auto_mode_cb);
    ros::Subscriber match_data_sub = n.subscribe("match_data", 1, &match_data_cb);

    ROS_WARN("Auto Client loaded");
    ros::Duration(3).sleep();
    ROS_WARN("post sleep");
    generateTrajectory(2, 3, 0);

    ROS_WARN("SUCCESS IN autoInterpreterClient.cpp");
    //ros::spin();
    return 0;
}
