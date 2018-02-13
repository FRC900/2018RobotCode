#include "ros/ros.h"
#include "ros_control_boilerplate/AutoMode.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
void auto_modes(const ros_control_boilerplate::AutoMode::ConstPtr & AutoMode, const ros_control_boilerplate::MatchSpecificData::ConstPtr& MatchData) {
   ROS_INFO("WASSEOP");
   if(MatchData->isAutonomous) {
        puts("bleargh");
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
