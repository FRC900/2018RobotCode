#include "ros/ros.h"
#include "ros_control_boilerplate/AutoMode.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/IntakeLiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

static int startPos = -1;
static int autoMode = -1;
static double start_time;

static ros::Publisher IntakePub;
static ros::Publisher ElevatorPub;
static ros::Publisher ClampPub;
static ros::Publisher VelPub;

std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>> ac;
void auto_modes(const ros_control_boilerplate::AutoMode::ConstPtr & AutoMode, const ros_control_boilerplate::MatchSpecificData::ConstPtr& MatchData) {
    ROS_INFO("Mode: %d, Start Position: %d", AutoMode->mode, AutoMode->position);
    
    if(MatchData->isAutonomous && !MatchData->isDisabled) {
        if(!start_time) {
            start_time = ros::Time::now().toSec();
        }
        if(ros::Time::now().toSec() >= start_time+2){
            return;
        }
        double time_start_auto = ros::Time::now().toSec();
        elevator_controller::Intake IntakeMsg;
        elevator_controller::ElevatorControl ElevatorMsg;
        std_msgs::Bool ClampMsg;
        behaviors::IntakeLiftGoal goal;

        geometry_msgs::Twist vel;
        vel.linear.x = 2;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
        VelPub.publish(vel);
        /*
        if(AutoMode->mode==1) {
        //3 cube switch-scale-scale
            //0: Time 0: Go to switch config
            ros::Duration(0).sleep(); //TODO
            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = false;

            ElevatorPub.publish(ElevatorMsg);

            //1: Time 1: Release Clamp
            ros::Duration(1).sleep(); //TODO
            ClampMsg.data = false;
            ClampPub.publish(ClampMsg);

            //2: Time 1.5: drop and start intake && go to intake config
            ros::Duration(1.5).sleep(); //TODO
            goal.IntakeCube = true;

            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = true;
            ElevatorPub.publish(ElevatorMsg);

            ac->sendGoal(goal);

            //3: Linebreak sensor: Clamp && release intake && stop running intake
            bool success = ac->waitForResult(ros::Duration(15)); //TODO

            if(!success) {
                ROS_WARN("Failed to intake cube: TIME OUT");
            }
            IntakeMsg.power = 0;
            IntakePub.publish(IntakeMsg);

            ros::Duration(.2).sleep();

            ClampMsg.data = true;
            ClampPub.publish(ClampMsg);

            IntakeMsg.spring_state = 1; //out
            IntakePub.publish(IntakeMsg);
            
            //4: Success of command 3: go to default config && soft-in intake
            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = true;
            ElevatorPub.publish(ElevatorMsg);
            ros::Duration(.1).sleep(); //TODO

            IntakeMsg.spring_state = 2;
            IntakePub.publish(IntakeMsg);

            //5: Time 2: go to mid level scale
            ros::Duration(2).sleep();
            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = true;
            ElevatorPub.publish(ElevatorMsg);

            //6: Time 3: release Clamp
            ros::Duration(3).sleep(); //TODO
            ClampMsg.data = false;
            ClampPub.publish(ClampMsg);

            //7: Time 4: go to intake config && run intake
            ros::Duration(4).sleep(); // TODO
            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = true;
            ElevatorPub.publish(ElevatorMsg);

            goal.IntakeCube = true;
            ac->sendGoal(goal);

            //8: Linebreak sensor: Clamp && release intake && stop running intake
            success = ac->waitForResult(ros::Duration(15));
            
            if(!success) {
                ROS_ERROR("Failed to intake cube: TIME OUT");
            }
            IntakeMsg.power=0;
            IntakePub.publish(IntakeMsg);

            ClampMsg.data = true;
            ClampPub.publish(ClampMsg);

            IntakeMsg.spring_state = 1;
            IntakePub.publish(IntakeMsg);

            //9: Success of command 8: go to default config && soft-in intake
            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = true;
            ElevatorPub.publish(ElevatorMsg);

            IntakeMsg.spring_state = 2;
            IntakePub.publish(IntakeMsg);

            //10: Time 5: go to mid scale config
            ros::Duration(5).sleep(); //TODO
            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = true;
            ElevatorPub.publish(ElevatorMsg);

            //11: Time 6: release Clamp
            ros::Duration(6).sleep(); //TODO
            ClampMsg.data = false;
            ClampPub.publish(ClampMsg);
            //12: Time 7: go to intake config
            ros::Duration(7).sleep(); //TODO
            ElevatorMsg.x = -1; //TODO
            ElevatorMsg.y = -1; //TODO
            ElevatorMsg.up_or_down = true;
            ElevatorPub.publish(ElevatorMsg);
        }
        else if(AutoMode->mode==2) {
            //2 cube longway scale-scale
                    //0: Time 0: Go to mid-scale config && drop intake
                    //1: Time 1: Release Clamp
                    //2: Time 1.5: go to intake config && start intake
                    //3: Linebreak sensor: Clamp && release intake && stop running intake
                    //4: Success of command 3: go to mid scale config && soft-in intake
                    //5: Time 2: release Clamp
                    //6: Time 3: go to intake config && run intake
                    //7: Linebreak sensor: Clamp && release intake && stop running intake

        }
        else if(AutoMode->mode==3) {
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
        else if(AutoMode->mode==4) {

        }
        else if(AutoMode->mode==5) {

        }
        else if(AutoMode->mode==6) {

        }
        else if(AutoMode->mode==7) {

        }
        else if(AutoMode->mode==8) {

        }
        else if(AutoMode->mode==9) {

        }
        else if(AutoMode->mode==10) {

        }
        else if(AutoMode->mode==11) {

        }
        else if(AutoMode->mode==12) {

        }
        else{
            
        }
        */
    }
    else {
        if(autoMode == AutoMode->mode || startPos == AutoMode->position) {
            autoMode = AutoMode->mode;
            startPos = AutoMode->position;
            //TODO generate 4 motion profiles
        }
    }
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "Auto_state_subscriber");
    ros::NodeHandle n;

    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>>("AutoServer", true);
    ac->waitForServer(); 

    IntakePub = n.advertise<elevator_controller::Intake>("elevator/Intake", 1);
    ElevatorPub = n.advertise<elevator_controller::ElevatorControl>("elevator/cmd_pos", 1);
    ClampPub = n.advertise<std_msgs::Bool>("elevator/Clamp", 1);
    VelPub = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

    message_filters::Subscriber<ros_control_boilerplate::AutoMode> auto_mode_sub(n, "Autonomous_Mode", 20);
    message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> match_data_sub(n, "match_data", 20);
    typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::AutoMode, ros_control_boilerplate::MatchSpecificData> data_sync;
    message_filters::Synchronizer<data_sync> sync(data_sync(20), auto_mode_sub, match_data_sub);
    sync.registerCallback(boost::bind(&auto_modes, _1, _2));

    ros::spin();
    return 0;

}
