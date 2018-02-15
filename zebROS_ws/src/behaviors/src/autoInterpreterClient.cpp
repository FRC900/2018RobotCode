#include "ros/ros.h"
#include "ros_control_boilerplate/AutoMode.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/IntakeLiftAction.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/ElevatorControl.h"

static int startPos = -1;
static int autoMode = -1;
void auto_modes(const ros_control_boilerplate::AutoMode::ConstPtr & AutoMode, const ros_control_boilerplate::MatchSpecificData::ConstPtr& MatchData) {
    //ROS_INFO("Mode: %d, Start Position: %d", AutoMode->mode, AutoMode->position);
    if(MatchData->isAutonomous) {
        actionlib::SimpleActionClient<behaviors::IntakeLiftAction> ac("AutoServer", true);
        //Field config 1
        if(AutoMode->mode==1) {
            //3 cube switch-scale-scale
                //0: Time 0: Go to switch config && drop and start intake
                
                //1: Time 1: Release Clamp && go to default config
                //2: Time 1.5: go to intake config
                //3: Linebreak sensor: Clamp && release intake && stop running intake
                //4: Success of command 3: go to mid scale config && soft-in intake
                //5: Time 2: release Clamp
                //6: Time 3: go to intake config && run intake
                //7: Linebreak sensor: Clamp && release intake && stop running intake
                //8: Success of command 6: go to mid scale config && soft-in intake
                //9: Time 4: release Clamp
                //10: Time 5: go to intake config
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
    
    message_filters::Subscriber<ros_control_boilerplate::AutoMode> auto_mode_sub(n, "Autonomous_Mode", 5);
    message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> match_data_sub(n, "match_data", 5);
    typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::AutoMode, ros_control_boilerplate::MatchSpecificData> data_sync;
    message_filters::Synchronizer<data_sync> sync(data_sync(5), auto_mode_sub, match_data_sub);
    sync.registerCallback(boost::bind(&auto_modes, _1, _2));

    ros::spin();
    return 0;

}
