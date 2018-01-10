#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"

int evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr
&msg) {
    /*
        map left joystick+bumpers+triggers into twist
        map right joystick into elevator/pivot position?
        autoscale to specific elevator height and pivot angle for now...
         

    */
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "scaled_joystick_state_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ScaledJoystickVals", 1000, evaluateCommands);
    ros::spin();

    return 0;
}
