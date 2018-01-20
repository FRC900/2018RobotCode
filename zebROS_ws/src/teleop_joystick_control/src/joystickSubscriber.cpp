#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"

#include <math.h>



static ros::Publisher ScaledValPub;
static double dead_zone=.1, slow_mode=.33, max_speed=5, joystick_scale=3;
double dead_zoneCheck(double val) {
    if(fabs(val)<=dead_zone) {
        return 0;
    }
    return val;
}
void joystick(const ros_control_boilerplate::JoystickState::ConstPtr &msg) {
    /*Joystick value scaling and magic stuff */
    double leftStickX = msg->leftStickX;
    double leftStickY = msg->leftStickY;

    double  rightStickX = msg->rightStickX;
    double  rightStickY = msg->rightStickY;

    ROS_INFO_STREAM("right Y: " << rightStickY <<"right X: " << rightStickX );
    ROS_INFO("WE BROKE");
    bool buttonAButton = msg->buttonAButton;
    bool buttonAPress = msg->buttonAPress;
    bool buttonARelease = msg->buttonARelease;

    bool buttonBButton = msg->buttonBButton;
    bool buttonBPress = msg->buttonBPress;
    bool buttonBRelease = msg->buttonBRelease;

    bool buttonXButton = msg->buttonXButton;
    bool buttonXPress = msg->buttonXPress;
    bool buttonXRelease = msg->buttonXRelease;

    bool buttonYButton = msg->buttonYButton;
    bool buttonYPress = msg->buttonYPress;
    bool buttonYRelease = msg->buttonYRelease;

    bool bumperLeftButton = msg->bumperLeftButton;
    bool bumperLeftPress = msg->bumperLeftPress;
    bool bumperLeftRelease = msg->bumperLeftRelease;

    bool bumperRightButton = msg->bumperRightButton;
    bool bumperRightPress = msg->bumperRightPress;
    bool bumperRightRelease = msg->bumperRightRelease;

    bool buttonBackButton = msg->buttonBackButton;
    bool buttonBackPress = msg->buttonBackPress;
    bool buttonBackRelease = msg->buttonBackRelease;

    bool buttonStartButton = msg->buttonStartButton;
    bool buttonStartPress = msg->buttonStartPress;
    bool buttonStartRelease = msg->buttonStartRelease;

    bool stickLeftButton = msg->stickLeftButton;
    bool stickLeftPress = msg->stickLeftPress;
    bool stickLeftRelease = msg->stickLeftRelease;

    bool stickRightButton = msg->stickRightButton;
    bool stickRightPress = msg->stickRightPress;
    bool stickRightRelease = msg->stickRightRelease;
    bool directionUpButton = msg->directionUpButton;
    bool directionUpPress = msg->directionUpPress;
    bool directionUpRelease = msg->directionUpRelease;
    
    bool directionDownButton = msg->directionDownButton;
    bool directionDownPress = msg->directionDownPress;
    bool directionDownRelease = msg->directionDownRelease;

    bool directionLeftButton = msg->directionLeftButton;
    bool directionLeftPress = msg->directionLeftPress;
    bool directionLeftRelease = msg->directionLeftRelease;

    bool directionRightButton = msg->directionRightButton;
    bool directionRightPress = msg->directionRightPress;
    bool directionRightRelease = msg->directionRightRelease;

    double  leftTrigger = msg->leftTrigger;
    double  rightTrigger = msg->rightTrigger;

    double scaledLeftStickX = (pow(dead_zoneCheck(leftStickX), joystick_scale))*max_speed;
    double scaledLeftStickY = (0-pow(dead_zoneCheck(leftStickY), joystick_scale))*max_speed;

    double scaledRightStickX = (0-pow(dead_zoneCheck(rightStickX), joystick_scale));
    double scaledRightStickY = (0-pow(dead_zoneCheck(rightStickY), joystick_scale));
    if(bumperLeftButton == true) {
        scaledLeftStickX *= slow_mode;
        scaledLeftStickY *= slow_mode;
        
        scaledRightStickX *= slow_mode;
        scaledRightStickY *= slow_mode;
    
        leftTrigger *= slow_mode;
        rightTrigger *= slow_mode;
    }
    //ROS_INFO("scaledLetStickX: %f scaledLeftStickY: %f\n", scaledLeftStickX, scaledLeftStickY);
    ros::Rate loop_rate(10);
    while(ros::ok()) { //why...............
        ros_control_boilerplate::JoystickState msg;


        msg.leftStickX = scaledLeftStickX;
        msg.leftStickY = scaledLeftStickY;

        msg.rightStickX = scaledRightStickX;
        msg.rightStickY = scaledRightStickY;

        msg.leftTrigger = leftTrigger;
        msg.rightTrigger = rightTrigger;

        msg.buttonAButton = buttonAButton;
        msg.buttonAPress = buttonAPress;
        msg.buttonARelease = buttonARelease;

        msg.buttonBButton = buttonBButton;
        msg.buttonBPress = buttonBPress;
        msg.buttonBRelease = buttonBRelease;

        msg.buttonYButton = buttonYButton;
        msg.buttonYPress = buttonYPress;
        msg.buttonYRelease = buttonYRelease;

        msg.buttonXButton = buttonXButton;
        msg.buttonXPress = buttonXPress;
        msg.buttonXRelease = buttonXRelease;

        msg.bumperLeftButton = bumperLeftButton;
        msg.bumperLeftPress = bumperLeftPress;
        msg.bumperLeftRelease = bumperLeftRelease;

        msg.bumperRightButton = bumperRightButton;
        msg.bumperRightPress = bumperRightPress;
        msg.bumperRightRelease = bumperRightRelease;

        msg.buttonBackButton = buttonBackButton;
        msg.buttonBackPress = buttonBackPress;
        msg.buttonBackRelease = buttonBackRelease;

        msg.buttonStartButton = buttonStartButton;
        msg.buttonStartPress = buttonStartPress;
        msg.buttonStartRelease = buttonStartRelease;

        msg.stickLeftButton = stickLeftButton;
        msg.stickLeftPress = stickLeftPress;
        msg.stickLeftRelease = stickLeftRelease;

        msg.stickRightButton = stickRightButton;
        msg.stickRightPress = stickRightPress;
        msg.stickRightRelease = stickRightRelease;

       
        msg.directionUpButton = directionUpButton;
        msg.directionUpPress = directionUpPress;
        msg.directionUpRelease = directionUpRelease;

        msg.directionDownButton = directionDownButton;
        msg.directionDownPress = directionDownPress;
        msg.directionDownRelease = directionDownRelease;

        msg.directionLeftButton = directionLeftButton;
        msg.directionLeftPress = directionLeftPress;
        msg.directionLeftRelease = directionLeftRelease;

        msg.directionRightButton = directionRightButton;
        msg.directionRightPress = directionRightPress;
        msg.directionRightRelease = directionRightRelease;
        

        ScaledValPub.publish(msg);
        ros::spinOnce();
        //break;

        loop_rate.sleep();
        //break;
    }
}


int main(int argc, char **argv) {
    ROS_INFO("hi");
    ros::init(argc, argv, "param_loader");
    ROS_INFO("hi");
    ros::NodeHandle nh;
    ROS_INFO("hi");
    nh.param("dead_zone", dead_zone, .1);
    ROS_INFO("hi");
    nh.param("slow_mode", slow_mode, .33);
    ROS_INFO("hi");
    nh.param("max_speed", max_speed, 3.285);
    ROS_INFO("hi");
    nh.param("joystick_scale", joystick_scale, 3.0);
    ROS_INFO("hi");
    ros::init(argc, argv, "joystick_state_subscriber");
    ROS_INFO("hi1");
    ros::NodeHandle n;
    ROS_INFO("hi2");
    ros::Subscriber sub = n.subscribe("frcrobot/joystick_states", 1, joystick);
    ROS_INFO("hi3");

    ros::init(argc, argv, "ScaledJoystickVals");
    ros::NodeHandle n_;
    ScaledValPub = n_.advertise<ros_control_boilerplate::JoystickState>("ScaledJoystickVals", 1);

    ros::spin();

    return 0;
}
