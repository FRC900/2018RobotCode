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

std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>> ac;
void auto_modes(const ros_control_boilerplate::AutoMode::ConstPtr & AutoMode, const ros_control_boilerplate::MatchSpecificData::ConstPtr& MatchData) {
    
    if(MatchData->isAutonomous && !MatchData->isDisabled) {
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
            //0: Time 0: Go to switch config
            ros::Duration(0).sleep(); //TODO
            ElevatorSrv.request.x = switch_config_x;
            ElevatorSrv.request.y = switch_config_y;
            ElevatorSrv.request.up_or_down = false;
            ElevatorSrv.request.override_sensor_limits = false;
            ElevatorSrv.request.override_pos_limits = false;

            ElevatorService.call(ElevatorSrv);

            //1: Time 1: Release Clamp
            ros::Duration(1).sleep(); //TODO
            ClampSrv.request.data = false;
            ClampService.call(ClampSrv);

            //2: Time 1.5: drop and start intake && go to intake config
            ros::Duration(1.5).sleep(); //TODO
            goal.IntakeCube = true;

            ElevatorSrv.request.x = intake_config_x;
            ElevatorSrv.request.y = intake_config_y;
            ElevatorSrv.request.up_or_down = true;
            ElevatorService.call(ElevatorSrv);

            ac->sendGoal(goal);

            //3: Linebreak sensor: Clamp && release intake && stop running intake
            bool success = ac->waitForResult(ros::Duration(timeout)); //TODO

            if(!success) {
                ROS_WARN("Failed to intake cube: TIME OUT");
            }
            IntakeSrv.request.power = 0;
            IntakeService.call(IntakeSrv);

            ros::Duration(.2).sleep();

            ClampSrv.request.data = true;
            ClampService.call(ClampSrv);

            IntakeSrv.request.spring_state = 1; //out
            IntakeService.call(IntakeSrv);
            
            //4: Success of command 3: go to default config && soft-in intake
            ElevatorSrv.request.x = default_x;
            ElevatorSrv.request.y = default_y;
            ElevatorSrv.request.up_or_down = true;
            ElevatorService.call(ElevatorSrv);
            ros::Duration(.1).sleep(); //TODO

            IntakeSrv.request.spring_state = 2;
            IntakeService.call(IntakeSrv);

            //5: Time 2: go to mid level scale
            ros::Duration(2).sleep();
            ElevatorSrv.request.x = mid_scale_config_x;
            ElevatorSrv.request.y = mid_scale_config_y;
            ElevatorSrv.request.up_or_down = true;
            ElevatorService.call(ElevatorSrv);

            //6: Time 3: release Clamp
            ros::Duration(3).sleep(); //TODO
            ClampSrv.request.data = false;
            ClampService.call(ClampSrv);

            //7: Time 4: go to intake config && run intake
            ros::Duration(4).sleep(); // TODO
            ElevatorSrv.request.x = intake_config_x;
            ElevatorSrv.request.y = intake_config_y;
            ElevatorSrv.request.up_or_down = true;
            ElevatorService.call(ElevatorSrv);

            goal.IntakeCube = true;
            ac->sendGoal(goal);

            //8: Linebreak sensor: Clamp && release intake && stop running intake
            success = ac->waitForResult(ros::Duration(timeout));
            
            if(!success) {
                ROS_ERROR("Failed to intake cube: TIME OUT");
            }
            IntakeSrv.request.power=0;
            IntakeService.call(IntakeSrv);

            ClampSrv.request.data = true;
            ClampService.call(ClampSrv);

            IntakeSrv.request.spring_state = 1;
            IntakeService.call(IntakeSrv);

            //9: Success of command 8: go to default config && soft-in intake
            ElevatorSrv.request.x = default_x;
            ElevatorSrv.request.y = default_y;
            ElevatorSrv.request.up_or_down = true;
            ElevatorService.call(ElevatorSrv);

            IntakeSrv.request.spring_state = 2;
            IntakeService.call(IntakeSrv);

            //10: Time 5: go to mid scale config
            ros::Duration(5).sleep(); //TODO
            ElevatorSrv.request.x = mid_scale_config_x;
            ElevatorSrv.request.y = mid_scale_config_y;
            ElevatorSrv.request.up_or_down = true;
            ElevatorService.call(ElevatorSrv);

            //11: Time 6: release Clamp
            ros::Duration(6).sleep(); //TODO
            ClampSrv.request.data = false;
            ClampService.call(ClampSrv);
            //12: Time 7: go to intake config
            ros::Duration(7).sleep(); //TODO
            ElevatorSrv.request.x = intake_config_x;
            ElevatorSrv.request.y = intake_config_y;
            ElevatorSrv.request.up_or_down = true;
            ElevatorService.call(ElevatorSrv);
        }
        else if(AutoMode->mode[auto_mode-1]==2) {
            //2 cube longelevator_controller::bool_srve
            //0: Time 0: Go to default config && drop intake
            ros::Duration(0).sleep(); //TODO
            ElevatorSrv.request.x = default_x;
            ElevatorSrv.request.y = default_y;
            ElevatorSrv.request.up_or_down = false;
            ElevatorSrv.request.override_sensor_limits = false;
            ElevatorSrv.request.override_pos_limits = false;

            IntakeSrv.request.up = false;
            IntakeSrv.request.spring_state = 2; //soft_in

            ElevatorService.call(ElevatorSrv);
            IntakeService.call(IntakeSrv);
                    
            //1: Time 1: Go to mid scale config && Release Clamp
            ros::Duration(1).sleep(); //TODO
            ElevatorSrv.request.x = mid_scale_config_x;
            ElevatorSrv.request.y = mid_scale_config_y;
            ElevatorSrv.request.up_or_down = false;
            ElevatorService.call(ElevatorSrv);
            ros::Duration(.3).sleep(); //TODO

            ClampSrv.request.data = false;
            ClampService.call(ClampSrv);


            //2: Time 1.5: go to intake config && start intake
            ros::Duration(1).sleep(); //TODO
            ElevatorSrv.request.x = intake_config_x;
            ElevatorSrv.request.y = intake_config_y;
            ElevatorSrv.request.up_or_down = false;

            ElevatorService.call(ElevatorSrv);

            goal.IntakeCube = true;
            ac->sendGoal(goal);

            //3: Linebreak sensor: Clamp && release intake && stop running intake
            bool success = ac->waitForResult(ros::Duration(timeout)); //TODO
            IntakeSrv.request.power = 0; 
            IntakeService.call(IntakeSrv);

            if(!success) {
                ROS_ERROR("Failed to intake cube! TIME OUT");
            }

            ClampSrv.request.data = true;
            ClampService.call(ClampSrv);

            IntakeSrv.request.spring_state = 1; //out;
            IntakeService.call(IntakeSrv);

            //4: Success of command 3: go to default config && soft-in intake
            ElevatorSrv.request.x = default_x;
            ElevatorSrv.request.y = default_y;
            ElevatorSrv.request.up_or_down = false;
            ElevatorService.call(ElevatorSrv);
            
            ros::Duration(.2).sleep(); //TODO
            IntakeSrv.request.spring_state = 2; //soft in
            IntakeService.call(IntakeSrv);
           
            //5: Time 2: go to mid level scale config
            ros::Duration(2).sleep(); //TODO
            ElevatorSrv.request.x = mid_scale_config_x;
            ElevatorSrv.request.y = mid_scale_config_y;
            ElevatorSrv.request.up_or_down = false;
            ElevatorService.call(ElevatorSrv);

            //6: Time 2.5: release Clamp
            ros::Duration(2.5).sleep(); //TODO
            ClampSrv.request.data = false;
            ClampService.call(ClampSrv);
            //7: Time 3: go to intake config && run intake
            ros::Duration(3).sleep(); //TODO
            ElevatorSrv.request.x = intake_config_x;
            ElevatorSrv.request.y = intake_config_y;
            ElevatorSrv.request.up_or_down = false;
            ElevatorService.call(ElevatorSrv);
            
            goal.IntakeCube = true;
            ac->sendGoal(goal);
            
            success = ac->waitForResult(ros::Duration(timeout));

            //8: Linebreak sensor: Clamp && release intake && stop running intake
            IntakeSrv.request.power = 0;
            IntakeService.call(IntakeSrv);

            if(!success) {
                ROS_WARN("Failed to intake cube: TIME OUT");
            }

            ClampSrv.request.data = true;
            ClampService.call(ClampSrv);

            IntakeSrv.request.spring_state = 1; //out
            IntakeService.call(IntakeSrv);



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
