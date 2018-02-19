#include "ros/ros.h"
#include "ros_control_boilerplate/AutoMode.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/IntakeLiftAction.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/bool_srv.h"
#include "cstdlib"
static int startPos = -1;
static int auto_mode = -1;
static double start_time;

//static ros::Publisher IntakeService;
static ros::ServiceClient IntakeService;
static ros::ServiceClient ElevatorService;
static ros::ServiceClient ClampService;
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
static double autoStart;

bool defaultConfig(elevator_controller::ElevatorControlS srv) {
    srv.request.x = default_x;
    srv.request.y = default_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool intakeConfig(elevator_controller::ElevatorControlS srv) {
    srv.request.x = intake_config_x;
    srv.request.y = intake_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool switchConfig(elevator_controller::ElevatorControlS srv) {
    srv.request.x = switch_config_x;
    srv.request.y = switch_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool highScale(elevator_controller::ElevatorControlS srv) {
    srv.request.x = high_scale_config_x;
    srv.request.y = high_scale_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool midScale(elevator_controller::ElevatorControlS srv) {
    srv.request.x = mid_scale_config_x;
    srv.request.y = mid_scale_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool lowScale(elevator_controller::ElevatorControlS srv) {
    srv.request.x = low_scale_config_x;
    srv.request.y = low_scale_config_y;
    srv.request.override_pos_limits = false;
    srv.request.override_sensor_limits = false;
    return ElevatorService.call(srv);
}
bool stopIntake(elevator_controller::Intake srv) {
    srv.request.power=0;
    return IntakeService.call(srv);
}
bool releaseClamp(elevator_controller::bool_srv srv) {
    srv.request.data = false;
    return ClampService.call(srv);
}
bool clamp(elevator_controller::bool_srv srv) {
    srv.request.data = true;
    return ClampService.call(srv);
}
bool releaseIntake(elevator_controller::Intake srv) {
    srv.request.spring_state = 1;
    IntakeService.call(srv);
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>> ac;
void auto_modes(const ros_control_boilerplate::AutoMode::ConstPtr & AutoMode, const ros_control_boilerplate::MatchSpecificData::ConstPtr& MatchData) {
    
    if(MatchData->isAutonomous && !MatchData->isDisabled) {
        if(MatchData->allianceData != "") {
            if(!start_time) {
                start_time = ros::Time::now().toSec();
            }
            if(ros::Time::now().toSec() >= start_time+2){
                return;
            }
            double time_start_auto = ros::Time::now().toSec();
            elevator_controller::Intake IntakeSrv;
            elevator_controller::ElevatorControlS ElevatorSrv;
            elevator_controller::bool_srv ClampSrv;
            behaviors::IntakeLiftGoal goal;

    /////////////////TESTING/////////////////
            geometry_msgs::Twist vel;
            vel.linear.x = 2;
            vel.linear.y = 0;
            vel.linear.z = 0;
            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = 0;
            VelPub.publish(vel);
            return;
    /////////////////////////////////////////
            if(MatchData->allianceData=="rlr") {
                auto_mode = 1;
            }
            else if(MatchData->allianceData=="lrl") {
                auto_mode = 2;
            }
            else if(MatchData->allianceData=="rrr") {
                auto_mode = 3;
            }
            else if(MatchData->allianceData =="lll") {
                auto_mode = 4;
            }

            if(AutoMode->mode[auto_mode-1]==1) {
            //3 cube switch-scale-scale
                //0: Time 1: Go to switch config
                ros::Duration(0).sleep(); //TODO
                switchConfig(ElevatorSrv);
                //1: Time 2: Release Clamp
                ros::Duration(1).sleep(); //TODO
                releaseClamp(ClampSrv);

                //2: Time 3: drop and start intake && go to intake config
                ros::Duration(1.5).sleep(); //TODO
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
                midScale(ElevatorSrv); 
                ros::Duration(.1).sleep(); //TODO

                IntakeSrv.request.spring_state = 2;
                IntakeService.call(IntakeSrv);

                //6: Time 5: release Clamp
                ros::Duration(3).sleep(); //TODO
                releaseClamp(ClampSrv);

                //7: Time 6: go to intake config && run intake
                ros::Duration(4).sleep(); // TODO
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
                ros::Duration(5).sleep(); //TODO
                midScale(ElevatorSrv);
                IntakeSrv.request.spring_state=2; //soft_in
                IntakeService.call(IntakeSrv);
                
                //11: Time 8: release Clamp
                ros::Duration(6).sleep(); //TODO
                releaseClamp(ClampSrv);

                //12: Time 9: go to intake config && start intake
                ros::Duration(7).sleep(); //TODO
                intakeConfig(ElevatorSrv);

                goal.IntakeCube = true;
                ac->sendGoal(goal);
                //13: Linebreak sensor: Clamp and release cube
            }
            else if(AutoMode->mode[auto_mode-1]==2) {
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
            else if(AutoMode->mode[auto_mode-1]==3) {
                //2 cube longway scale-scale
                        //0: Time 0: Go to mid-scale config && drop intake
                        //1: Time 1: Release Clamp
                        //2: Time 1.5: go to intake config && start intake
                        //3: Linebreak sensor: Clamp && release intake && stop running intake
                        //4: Success of command 3: go to switch config && soft-in intake
                        //5: Time 2: release Clamp
                        //6: Time 3: go to intake config && run intake
                        //7: Linebreak sensor: Clamp && release intake && stop running intake
            }
            else if(AutoMode->mode[auto_mode-1]==4) {

            }
            else if(AutoMode->mode[auto_mode-1]==5) {

            }
            else if(AutoMode->mode[auto_mode-1]==6) {

            }
            else if(AutoMode->mode[auto_mode-1]==7) {

            }
            else if(AutoMode->mode[auto_mode-1]==8) {

            }
            else if(AutoMode->mode[auto_mode-1]==9) {

            }
            else if(AutoMode->mode[auto_mode-1]==10) {

            }
            else if(AutoMode->mode[auto_mode-1]==11) {

            }
            else if(AutoMode->mode[auto_mode-1]==12) {

            }
            else{
                
            }
        }
        else {
            if(!autoStart) {
                autoStart = ros::Time::now().toSec();
            }
            else {
                if(ros::Time::now().toSec() > autoStart + 5) {
                    //RUN CROSS LINE AUTO
                }
            }   
        }   
    }
    else {
        if(auto_mode == AutoMode->mode[auto_mode-1] || startPos == AutoMode->position) {
            auto_mode = AutoMode->mode[auto_mode-1];
            startPos = AutoMode->position;
            //TODO generate 4 motion profiles
        }
    }
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "Auto_state_subscriber");
    ros::NodeHandle n;

    ros::NodeHandle n_params(n, "teleop_params");
    
    n_params.getParam("high_scale_config_x", high_scale_config_x);
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

    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>>("auto_Interpreter_Server", true);
    ac->waitForServer(); 

    //IntakeService = n.advertise<elevator_controller::Intake>("elevator/Intake", 1);
    IntakeService = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
    //ElevatorService = n.advertise<elevator_controller::ElevatorControl>("elevator/cmd_pos", 1);
    ElevatorService = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
    //ClampService = n.advertise<std_msgs::Bool>("elevator/Clamp", 1);
    ClampService = n.serviceClient<elevator_controller::bool_srv>("/frcrobot/elevator_controller/clamp");
    VelPub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);

    message_filters::Subscriber<ros_control_boilerplate::AutoMode> auto_mode_sub(n, "Autonomous_Mode", 20);
    message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> match_data_sub(n, "match_data", 20);
    typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::AutoMode, ros_control_boilerplate::MatchSpecificData> data_sync;
    message_filters::Synchronizer<data_sync> sync(data_sync(20), auto_mode_sub, match_data_sub);
    sync.registerCallback(boost::bind(&auto_modes, _1, _2));

    ros::spin();
    return 0;

}
