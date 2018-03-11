#include <behaviors/autoInterpreterClient.h>

ros::ServiceClient point_gen;
ros::ServiceClient swerve_control;


static int start_pos = 0;
std::vector<int> auto_mode_vect = {0, 0, 0, 0};
std::vector<double> delays_vect = {0, 0, 0, 0};
static int auto_mode = -1;
double start_time;
double curr_time;
double last_time;

//static ros::Publisher IntakeService;
static ros::ServiceClient IntakeService;
static ros::ServiceClient ElevatorService;
static ros::ServiceClient ClampService;
static ros::ServiceClient BrakeService;
static ros::Publisher VelPub;

static double high_scale_config_x;
static double high_scale_config_y;
static double mid_scale_config_x;
static double mid_scale_config_y;
static double low_scale_config_x;
static double low_scale_config_y;
static double switch_config_x;
static double switch_config_y;
static double exchange_config_x;
static double exchange_config_y;
static double intake_config_x;
static double intake_config_y;
static double default_x;
static double default_y;
static double timeout;
static double delay;
static double autoStart = -1;
static int layout;
static XmlRpc::XmlRpcValue modes;
static std::vector<trajectory_msgs::JointTrajectory> trajectories;

static bool in_auto;

std::vector<std::vector<double>> times_vect(4);
std::vector<talon_swerve_drive_controller::FullGenCoefs> coefs_vect(4);


// TODO : for all of these just make srv a local
bool defaultConfig(elevator_controller::ElevatorControlS srv) {
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
bool intakeConfig(elevator_controller::ElevatorControlS srv) {
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
bool switchConfig(elevator_controller::ElevatorControlS srv) {
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
bool highScale(elevator_controller::ElevatorControlS srv) {
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
bool midScale(elevator_controller::ElevatorControlS srv) {
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
bool lowScale(elevator_controller::ElevatorControlS srv) {
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
bool stopIntake(elevator_controller::Intake srv) {
    srv.request.power=0;
    if (!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in stopIntake");
		return false;
	}
	return true;
}
bool releaseClamp(elevator_controller::bool_srv srv) {
    srv.request.data = false;
    if (!ClampService.call(srv))
	{
		ROS_ERROR("Service call failed : ClampService in releaseClamp");
		return false;
	}
	return true;
}
bool clamp(elevator_controller::bool_srv srv) {
    srv.request.data = true;
    if (!ClampService.call(srv))
	{
		ROS_ERROR("Service call failed : ClampService in clamp");
		return false;
	}
	return true;
}
bool releaseIntake(elevator_controller::Intake srv) {
    srv.request.spring_state = 1;
    srv.request.power = 0;
    if (!IntakeService.call(srv))
	{
		ROS_ERROR("Service call failed : IntakeService in releaseIntake");
		return false;
	}
	return true;
}


void generateTrajectory(int auto_mode, int layout, int start_pos) {

    //XmlRpc::XmlRpcValue &path = modes[auto_mode][layout][start_pos];
    XmlRpc::XmlRpcValue &path = modes[auto_mode][layout][start_pos];
    //XmlRpc::XmlRpcValue &xml_x = path["x"];
    //XmlRpc::XmlRpcValue &xml_t = path["times"];
    ROS_WARN("check 0");
    const int num_splines = path.size();
    ROS_WARN("check 1");
    talon_swerve_drive_controller::FullGenCoefs srv;

	ROS_INFO_STREAM("checking size: " << num_splines << " is real?");
    for(int num = 0; num<num_splines; num++) {
        XmlRpc::XmlRpcValue &spline = path[num];
        ROS_WARN("check 1.5");
        XmlRpc::XmlRpcValue &x = spline["x"];
        ROS_WARN("check 2");
        XmlRpc::XmlRpcValue &y = spline["y"];
        ROS_WARN("check 3");
        XmlRpc::XmlRpcValue &orient = spline["orient"];
        ROS_WARN("check 4");
        XmlRpc::XmlRpcValue &time = spline["time"];

        talon_swerve_drive_controller::Coefs x_coefs;
        talon_swerve_drive_controller::Coefs y_coefs;
        talon_swerve_drive_controller::Coefs orient_coefs;
        ROS_WARN("check 5");
        for(int i = 0; i<x.size(); i++) {
            ROS_WARN("check 6");
            const double x_coef = x[i];
            const double y_coef = y[i];
            const double orient_coef = orient[i];

            //ROS_WARN("%f", orient_coef);
            x_coefs.spline.push_back(x_coef);
            y_coefs.spline.push_back(y_coef);
            orient_coefs.spline.push_back(orient_coef);
            ROS_WARN("check 7");
        }
        const double t = time;
        times_vect[layout].push_back(t);
        /*
        if(coefs_vect[layout].request.x_coefs.size()!=0) {
            coefs_vect[layout].request.x_coefs.clear();
            coefs_vect[layout].request.y_coefs.clear();
            coefs_vect[layout].request.orient_coefs.clear();
            coefs_vect[layout].request.end_points.clear();
        }*/
        srv.request.x_coefs.push_back(x_coefs);
        srv.request.y_coefs.push_back(y_coefs);
        srv.request.orient_coefs.push_back(orient_coefs);
        srv.request.end_points.push_back(num+1);
    }
     
    ROS_WARN("check 10");
    /*for(int i = 0; i<xml_t.size(); i++) {
        const double t_val = xml_t[i];
        times_vect[layout].push_back(t_val);
    }*/
    srv.request.initial_v = 0;
    srv.request.final_v = 0;
    coefs_vect[layout] = srv;
    ROS_WARN("check 11");
    if (!point_gen.call(coefs_vect[layout]))
		ROS_ERROR("point_gen call failed in autoInterpreterClient generateTrajectory()");
    
    ///JUST FOR TESTING_______
    bufferTrajectory(layout);
    ///JUST FOR TESTING_______
}

std::string lower(std::string str) {
    std::string new_string;
    for(int i = 0; i<str.size(); i++) {
       new_string += (tolower(str[i])); 
    }
    return new_string;
}

void bufferTrajectory(int auto_mode) {
    ROS_WARN("Run trajectory");
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.points = coefs_vect[auto_mode].response.points;
    ROS_INFO_STREAM("num_points: " << coefs_vect[auto_mode].response.points.size() << " dt: "<< coefs_vect[auto_mode].response.dt);
    
    swerve_control_srv.request.dt = coefs_vect[auto_mode].response.dt;
    swerve_control_srv.request.buffer = true;
    swerve_control_srv.request.clear  = true;
    swerve_control_srv.request.run    = false;
    swerve_control_srv.request.mode   = true;
    
    if (!swerve_control.call(swerve_control_srv))
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient generateTrajectory()");
}
void runTrajectory(int auto_mode) {
    ROS_WARN("Run trajectory");
    talon_swerve_drive_controller::MotionProfilePoints swerve_control_srv;
    swerve_control_srv.request.buffer = false;
    swerve_control_srv.request.clear  = false;
    swerve_control_srv.request.run    = true;
    swerve_control_srv.request.mode   = true;
    
    if (!swerve_control.call(swerve_control_srv))
		ROS_ERROR("swerve_control call() failed in autoInterpreterClient generateTrajectory()");
}

void auto_mode_cb(const ros_control_boilerplate::AutoMode::ConstPtr &AutoMode) {
    if(AutoMode->position != start_pos) {
        for(int i = 0; i<4; i++) {
            if(AutoMode->mode[i] == 3 || AutoMode->mode[i] == 4) {
                generateTrajectory(i, AutoMode->mode[i]-3, AutoMode->position);
            }
            auto_mode_vect[i] = AutoMode->mode[i];
            delays_vect[i] = AutoMode->delays[i];
        }
    }
    else {
        for(int i = 0; i<4; i++) {
            if(AutoMode->mode[i] != auto_mode_vect[i] && (AutoMode->mode[i] == 3 || AutoMode->mode[i] == 4)) {
                generateTrajectory(i, AutoMode->mode[i]-3, AutoMode->position);
            }
            auto_mode_vect[i] = AutoMode->mode[i];
            delays_vect[i] = AutoMode->delays[i];
        }
    }
    start_pos = AutoMode->position;

}

void match_data_cb(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {
    if(MatchData->isEnabled && MatchData->isAutonomous && MatchData->allianceData != "") {
        std::vector<double> times;
        if(lower(MatchData->allianceData)=="rlr") {
            auto_mode = 0;
            layout = 1;
            for(int i = 0; i<times_vect[0].size(); i++) {
                //times.push_back(times_vect[0][i]);
            }
        }
        else if(lower(MatchData->allianceData)=="lrl") {
    //ROS_WARN("auto entered");
        //ROS_WARN("2");
            auto_mode = 1;
            layout = 1;
            for(int i = 0; i<times_vect[1].size(); i++) {
                //times.push_back(times_vect[1][i]);
            }
        }
        else if(lower(MatchData->allianceData)=="rrr") {
            auto_mode = 2;
            layout = 2;
            for(int i = 0; i<times_vect[2].size(); i++) {
                //times.push_back(times_vect[2][i]);
            }
        }
        else if(lower(MatchData->allianceData) =="lll") {
    //ROS_WARN("auto entered");
        //ROS_WARN("3");
            auto_mode = 3;
            layout = 2;
            for(int i = 0; i<times_vect[3].size(); i++) {
                //times.push_back(times_vect[3][i]);
            }
        }
        in_auto = true;
        run_auto(auto_mode);
    }
    else if (MatchData->allianceData != "") {
        std::vector<double> times;
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
        bufferTrajectory(auto_mode);
        
    }
    else {
        in_auto = false;
    }
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::RobotAction>> ac;
void run_auto(int auto_mode) {
    std::vector<double> times = times_vect[auto_mode];
    elevator_controller::Intake IntakeSrv;
    elevator_controller::ElevatorControlS ElevatorSrv;
    elevator_controller::bool_srv ClampSrv;
    behaviors::RobotGoal robot_goal;
    ROS_WARN("auto entered");
    ros::Rate r(10);
    start_time = ros::Time::now().toSec();
    while(in_auto && ros::Time::now().toSec() < start_time + delays_vect[auto_mode]) {
        r.sleep(); 
        ros::spinOnce();
    }
    start_time = ros::Time::now().toSec();
    if(auto_mode_vect[auto_mode] == 1) {
        while(in_auto) {
            ROS_WARN("Basic drive forward auto");
            //basic drive forward cmd_vel
            geometry_msgs::Twist vel;
            vel.linear.z = 0;
            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = 0;

            switchConfig(ElevatorSrv); 
            if(auto_mode == 1 || auto_mode == 3) { //if goal is on the right
               if(start_pos == 0) {
                   delay = 6; //TODO
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = 0.12; //positive y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
               }
               if(start_pos == 2) {
                   delay = 6; //TODO
                   while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = -0.12; //negative y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
                else {
                    delay = 3.04; //TODO
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
                if(start_pos == 0){
                    delay = 6; 
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = 0.12; //positive y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
                if(start_pos == 2) {
                    delay = 6; //TODO
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 1.05; //positive x a lot
                        vel.linear.y = -0.12; //negative y a little bit
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
                else {
                    delay = 3.04; //TODO
                    while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                        vel.linear.x = 0.875; //positive x some
                        vel.linear.y = -0.3; //negative y some
                        VelPub.publish(vel);
                        r.sleep();
                        ros::spinOnce();
                    }
                }
            }
            ROS_WARN("braked");
            talon_swerve_drive_controller::Blank blank;
            blank.request.nothing = true;
            BrakeService.call(blank);
            in_auto = false;
        }
    }
    if(auto_mode_vect[auto_mode] == 2) {
        ROS_WARN("Basic switch auto mode");
        //basic switch cmd_vel
        geometry_msgs::Twist vel;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        double start_time = ros::Time::now().toSec();
        switchConfig(ElevatorSrv); 
        if(auto_mode == 1 || auto_mode == 3) { //if goal is on the right
           if(start_pos == 0) {
               delay = 3.5; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.05;
                    vel.linear.y = -1.5;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
            }
           if(start_pos == 2) {
            delay = 3; //TODO
               while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.2;
                    vel.linear.y = .53;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
           }
            else {
                delay = 3; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.75;
                    vel.linear.y = -0.7; 
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
                ROS_WARN("braked");
                talon_swerve_drive_controller::Blank blank;
                blank.request.nothing = true;
                if (!BrakeService.call(blank))
					ROS_ERROR("BrakeService call failed in autoMode == 1");
            }
            releaseClamp(ClampSrv);
        }
        else if(auto_mode == 2 || auto_mode == 4) { //goal is on the left
            if(start_pos == 0){
                delay = 3; 
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.2;
                    vel.linear.y = -.53;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
                ROS_WARN("braked");
                talon_swerve_drive_controller::Blank blank;
                blank.request.nothing = true;
                if (!BrakeService.call(blank))
					ROS_ERROR("BrakeService call failed in autoMode == 2");
            }
            if(start_pos == 2) {
                delay = 3.5; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.05;
                    vel.linear.y = 1.5;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
            }
            else {
                delay = 3; //TODO
                while(ros::Time::now().toSec() < start_time + delay && in_auto) {
                    vel.linear.x = 1.5;
                    vel.linear.y = .75;
                    VelPub.publish(vel);
                    r.sleep();
                    ros::spinOnce();
                }
            }
            releaseClamp(ClampSrv);
        }
        ROS_WARN("braked");
        talon_swerve_drive_controller::Blank blank;
        blank.request.nothing = true;
        BrakeService.call(blank);
        in_auto = false;
        
    }
    
    if(auto_mode_vect[auto_mode] == 3) {
        ROS_WARN("Profiled Scale");
        runTrajectory(auto_mode);
        while(in_auto) { 
            //Profiled scale
            curr_time = ros::Time::now().toSec();
            if(curr_time > times[0] && curr_time <= times[0] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale elevator to mid reached");
                midScale(ElevatorSrv);
            }
            if(curr_time > times[1] && curr_time <= times[1] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale release clamp reached");
                releaseClamp(ClampSrv);
            }
            last_time = curr_time;
            r.sleep();
            ros::spinOnce();
        }
        ROS_WARN("braked");
        talon_swerve_drive_controller::Blank blank;
        blank.request.nothing = true;
        BrakeService.call(blank);
        in_auto = false;
    }
    if(auto_mode_vect[auto_mode] == 4) {
        ROS_WARN("Profiled Scale");
        runTrajectory(auto_mode);
        while(in_auto) { 
            //Profiled scale
            curr_time = ros::Time::now().toSec();
            if(curr_time > times[0] && curr_time <= times[0] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale elevator to mid reached");
                midScale(ElevatorSrv);
            }
            if(curr_time > times[1] && curr_time <= times[1] + (curr_time-last_time)) {
                ROS_WARN("Profiled Scale release clamp reached");
                releaseClamp(ClampSrv);
            }
            if(curr_time > times[2] && curr_time <= times[2] + (curr_time-last_time)) {
                ROS_WARN("Intaking Cube and going to intake config");
                robot_goal.IntakeCube = true; 
            }
            last_time = curr_time;
            r.sleep();
            ros::spinOnce();
        }
        ROS_WARN("braked");
        talon_swerve_drive_controller::Blank blank;
        blank.request.nothing = true;
        BrakeService.call(blank);
        in_auto = false;
    }
    /*
    if(AutoMode->mode[auto_mode]==1) {
    //3 cube switch-scale-scale
        //0: Time 1: Go to switch config
        ros::Duration(times[0]).sleep(); //TODO
        switchConfig(ElevatorSrv);
        //1: Time 2: Release Clamp
        ros::Duration(times[1]).sleep(); //TODO
        releaseClamp(ClampSrv);

        //2: Time 3: drop and start intake && go to intake config
        ros::Duration(times[2]).sleep(); //TODO
        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //3: Linebreak sensor: Clamp && release intake && stop running intake

        bool success = ac->waitForResult(ros::Duration(timeout)); //TODO

        if(!success) {
            ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake(IntakeSrv);

        intakeConfig(ElevatorSrv); 
        ros::Duration(.2).sleep();

        clamp(ClampSrv);

        releaseIntake(IntakeSrv); 
        
        //4: Time 4: go to scale config && soft-in intake
        ros::Duration(times[3]).sleep();
        midScale(ElevatorSrv); 
        r.sleep() //TODO

        IntakeSrv.request.spring_state = 2;
        IndtakeService.call(IntakeSrv);

        //6: Time 5: release Clamp
        ros::Duration(times[4]).sleep(); //TODO
        releaseClamp(ClampSrv);

        //7: Time 6: go to intake config && run intake
        ros::Duration(times[5]).sleep(); // TODO
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //8: Linebreak sensor: Clamp && release intake && stop running intake
        success = ac->waitForResult(ros::Duration(timeout));
        
        if(!success) {
            ROS_ERROR("Failed to intake cube: TIME OUT");
        }
        stopIntake(IntakeSrv);

        clamp(ClampSrv);

        releaseIntake(IntakeSrv);
        //10: Time 7: go to mid scale config && soft in intake
        ros::Duration(times[6]).sleep(); //TODO
        midScale(ElevatorSrv);
        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call(IntakeSrv);
        
        //11: Time 8: release Clamp
        ros::Duration(times[7]).sleep(); //TODO
        releaseClamp(ClampSrv);

        //12: Time 9: go to intake config && start intake
        ros::Duration(times[8]).sleep(); //TODO
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);
        //13: Linebreak sensor: Clamp and release cube
        clamp(ClampSrv);
        releaseIntake(IntakeSrv);
    }
    else if(AutoMode->mode[auto_mode]==2) {
        //2 cube long elevator_controller::bool_srve
        //0: Time 0: Go to default config && drop intake
        ros::Duration(0).sleep(); //TODO
        defaultConfig(ElevatorSrv);

        IntakeSrv.request.up = false;
        IntakeSrv.request.spring_state = 2; //soft_in

        IntakeService.call(IntakeSrv);
                
        //1: Time 1: Go to mid scale config && Release Clamp
        ros::Duration(1).sleep(); //TODO
        midScale(ElevatorSrv);
        ros::Duration(.3).sleep(); //TODO

        releaseClamp(ClampSrv);

        //2: Time 1.5: go to intake config && start intake
        ros::Duration(1).sleep(); //TODO
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //3: Linebreak sensor: Clamp && release intake && stop running intake
        bool success = ac->waitForResult(ros::Duration(timeout)); //TODO
        stopIntake(IntakeSrv);

        if(!success) {
            ROS_ERROR("Failed to intake cube! TIME OUT");
        }

        clamp(ClampSrv);

        releaseIntake(IntakeSrv);

        //4: Success of command 3: go to default config && soft-in intake
        defaultConfig(ElevatorSrv); 
        ros::Duration(.2).sleep(); //TODO
        IntakeSrv.request.spring_state = 2; //soft in
        IntakeService.call(IntakeSrv);
       
        //5: Time 2: go to mid level scale config
        ros::Duration(2).sleep(); //TODO
        midScale(ElevatorSrv);

        //6: Time 2.5: release Clamp
        ros::Duration(2.5).sleep(); //TODO
        releaseClamp(ClampSrv);

        //7: Time 3: go to intake config && run intake
        ros::Duration(3).sleep(); //TODO
        intakeConfig(ElevatorSrv); 

        goal.IntakeCube = true;
        ac->sendGoal(goal);
        
        success = ac->waitForResult(ros::Duration(timeout));

        //8: Linebreak sensor: Clamp && release intake && stop running intake
        stopIntake(IntakeSrv);
        if(!success) {
            ROS_WARN("Failed to intake cube: TIME OUT");
        }

        clamp(ClampSrv);
        releaseIntake(IntakeSrv);


    }
    else if(AutoMode->mode[auto_mode]==3) {
    //3 cube switch-scale-scale
        //0: Time 1: Go to mid scale config
        ros::Duration(times[0]).sleep(); //TODO
        midScale(ElevatorSrv);
        //1: Time 2: Release Clamp
        ros::Duration(times[1]).sleep(); //TODO
        releaseClamp(ClampSrv);

        //2: Time 3: drop and start intake && go to intake config
        ros::Duration(times[2]).sleep(); //TODO
        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //3: Linebreak sensor: Clamp && release intake && stop running intake

        bool success = ac->waitForResult(ros::Duration(timeout)); //TODO

        if(!success) {
            ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake(IntakeSrv);

        intakeConfig(ElevatorSrv); 
        ros::Duration(.2).sleep();

        clamp(ClampSrv);

        releaseIntake(IntakeSrv); 
        
        //4: Time 4: go to switch config && soft-in intake
        ros::Duration(times[3]).sleep();
        switchConfig(ElevatorSrv); 
        r.sleep() //TODO

        IntakeSrv.request.spring_state = 2;
        IntakeService.call(IntakeSrv);

        //6: Time 5: release Clamp
        ros::Duration(times[4]).sleep(); //TODO
        releaseClamp(ClampSrv);

        //7: Time 6: go to intake config && run intake
        ros::Duration(times[5]).sleep(); // TODO
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //8: Linebreak sensor: Clamp && release intake && stop running intake
        success = ac->waitForResult(ros::Duration(timeout));
        
        if(!success) {
            ROS_ERROR("Failed to intake cube: TIME OUT");
        }
        stopIntake(IntakeSrv);

        clamp(ClampSrv);

        releaseIntake(IntakeSrv);
        //10: Time 7: go to mid scale config && soft in intake
        ros::Duration(times[6]).sleep(); //TODO
        midScale(ElevatorSrv);
        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call(IntakeSrv);
        
        //11: Time 8: release Clamp
        ros::Duration(times[7]).sleep(); //TODO
        releaseClamp(ClampSrv);

        //12: Time 9: go to intake config && start intake
        ros::Duration(times[8]).sleep(); //TODO
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);
        //13: Linebreak sensor: Clamp and release cube
        clamp(ClampSrv);
        releaseIntake(IntakeSrv);
    
}
    else if(AutoMode->mode[auto_mode]==4) { //TODO fix times
    //4 cube scale-scale-scale-switch
        //1: Time 1: scale config and soft-in intake
        ros::Duration(times[0]).sleep(); //good
        midScale(ElevatorSrv);

        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call(IntakeSrv);

        //2: Time 2: Release clamp
        ros::Duration(times[4]).sleep(); //good
        releaseClamp(ClampSrv);

        //3: Time 3: intake config and run intake
        ros::Duration(times[5]).sleep();
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //4: Linebreak sensor: Clamp, release intake, stop running intake
        bool success = ac->waitForResult(ros::Duration(timeout));

        if(!success) {
            ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake(IntakeSrv);

        intakeConfig(ElevatorSrv); 
        ros::Duration(.2).sleep();

        clamp(ClampSrv);

        releaseIntake(IntakeSrv); 

        //5: Time next: go to scale config and soft-in intake 
        ros::Duration(times[0]).sleep();
        midScale(ElevatorSrv);

        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call(IntakeSrv);

        //6: Time 1: Release clamp
        ros::Duration(times[4]).sleep();
        releaseClamp(ClampSrv);

        //7: Time 2: intake config and run intake
        ros::Duration(times[5]).sleep();
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //8: Linebreak sensor: clamp, release intake, stop running intake
        success = ac->waitForResult(ros::Duration(timeout));

        if(!success) {
            ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake(IntakeSrv);

        intakeConfig(ElevatorSrv); 
        ros::Duration(.2).sleep();

        clamp(ClampSrv);

        releaseIntake(IntakeSrv); 

        //9: Time next: go to scale config and soft-in intake 
        ros::Duration(times[0]).sleep();
        midScale(ElevatorSrv);

        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call(IntakeSrv);

        //10: Time 1: Release clamp
        ros::Duration(times[4]).sleep();
        releaseClamp(ClampSrv);

        //11: Time 2: intake config and run intake
        ros::Duration(times[5]).sleep();
        intakeConfig(ElevatorSrv);

        goal.IntakeCube = true;
        ac->sendGoal(goal);

        //12: Linebreak sensor: clamp, release intake, stop running intake
        success = ac->waitForResult(ros::Duration(timeout));

        if(!success) {
            ROS_WARN("Failed to intake cube: TIME OUT");
        }

        stopIntake(IntakeSrv);

        intakeConfig(ElevatorSrv); 
        ros::Duration(.2).sleep();

        clamp(ClampSrv);

        releaseIntake(IntakeSrv);

        //13: Time something: switch config and soft-in intake
        ros::Duration(times[0]).sleep();
        switchConfig(ElevatorSrv);

        IntakeSrv.request.spring_state=2;
        IntakeService.call(IntakeSrv);

        //14: Time something: release clamp
        ros::Duration(times[4]).sleep();
        releaseClamp(ClampSrv);

    }
    else if(AutoMode->mode[auto_mode]==5) {
    //1 cube simple switch
        //1: Time 1: Go to switch config and soft_in intake
        ros::Duration(times[0]).sleep();
        switchConfig(ElevatorSrv);

        ros::Duration(times[1]).sleep();
        IntakeSrv.request.spring_state=2; //soft_in
        IntakeService.call(IntakeSrv);

        //2: Time 2: Release clamp
        ros::Duration(times[2]).sleep();
        releaseClamp(ClampSrv);
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
<<<<<<< HEAD
    else if(AutoMode->mode[auto_mode]==12) {

=======
    else if(AutoMode->mode[auto_mode-1]==12) {
>>>>>>> ee60b59ed7844426bcf5ea1c1d34f38fc65e8fa9
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
   
    n_params.getParam("high_scale_config_y", high_scale_config_y);
    n_params.getParam("mid_scale_config_x", mid_scale_config_x);
    n_params.getParam("mid_scale_config_y", mid_scale_config_y);
    n_params.getParam("low_scale_config_x", low_scale_config_x);
    n_params.getParam("low_scale_config_y", low_scale_config_y);
    n_params.getParam("switch_config_x", switch_config_x);
    n_params.getParam("switch_config_y", switch_config_y);
    n_params.getParam("exchange_config_x", exchange_config_x);
    n_params.getParam("exchange_config_y", exchange_config_y);
    n_params.getParam("intake_config_x", intake_config_x);
    n_params.getParam("intake_config_y", intake_config_y);
    n_params.getParam("default_x", default_x);
    n_params.getParam("default_y", default_y);
    n_params.getParam("timeout", timeout);

    n_params_behaviors.getParam("modes", modes);
    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::RobotAction>>("auto_interpreter_server", true);
    ac->waitForServer(); 

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	point_gen = n.serviceClient<talon_swerve_drive_controller::FullGenCoefs>("/point_gen/command", true, service_connection_header);
	swerve_control = n.serviceClient<talon_swerve_drive_controller::MotionProfilePoints>("/frcrobot/swerve_drive_controller/run_profile", false, service_connection_header);

    //IntakeService = n.advertise<elevator_controller::Intake>("elevator/Intake", 1);
    IntakeService = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
    //ElevatorService = n.advertise<elevator_controller::ElevatorControl>("elevator/cmd_pos", 1);
    ElevatorService = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
    //ClampService = n.advertise<std_msgs::Bool>("elevator/Clamp", 1);
    ClampService = n.serviceClient<elevator_controller::bool_srv>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
    BrakeService = n.serviceClient<talon_swerve_drive_controller::Blank>("/frcrobot/talon_swerve_drive_controller/brake", false, service_connection_header);
    
    VelPub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);

    ros::Subscriber auto_mode_sub = n.subscribe("autonomous_mode", 1, &auto_mode_cb);
    ros::Subscriber match_data_sub = n.subscribe("match_data", 1, &match_data_cb);
    ROS_WARN("Auto Client loaded");
    ros::Duration(10).sleep();
    ROS_WARN("post sleep");
    generateTrajectory(0, 2, 0);

    ROS_WARN("SUCCESS IN autoInterpreterClient.cpp");
    ros::spin();
    return 0;
}
