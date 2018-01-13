#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"

#include <math.h>

#define DEAD .1
#define SLOW .3

double deadzone(double val) {
    if(fabs(val)<=DEAD) {
        return 0;
    }
    return val;
}

static ros::Publisher ScaledValPub;
void joystick(const ros_control_boilerplate::JoystickState::ConstPtr &msg) {
    /*Joystick value scaling and magic stuff */
    double leftStickX = msg->leftStickX;
    double leftStickY = msg->leftStickY;

    double  rightStickX = msg->rightStickX;
    double  rightStickY = msg->rightStickY;

    double  leftTrigger = msg->leftTrigger;
    double  rightTrigger = msg->rightTrigger;

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
/*
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
*/
    double scaledLeftStickX = 0-pow(deadzone(leftStickX), 3);
    double scaledLeftStickY = 0-pow(deadzone(leftStickY), 3);

    double scaledRightStickX = 0-pow(deadzone(rightStickX), 3);
    double scaledRightStickY = 0-pow(deadzone(rightStickY), 3);
    if(bumperLeftButton == true) {
        scaledLeftStickX *= SLOW;
        scaledLeftStickY *= SLOW;
        
        scaledRightStickX *= SLOW;
        scaledRightStickY *= SLOW;
    
        leftTrigger *= SLOW;
        rightTrigger *= SLOW;
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

        /*
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
        */

        ScaledValPub.publish(msg);
        ros::spinOnce();
        break;

        loop_rate.sleep();
        break;
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "joystick_state_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joystick_states", 1000, joystick);

    ros::init(argc, argv, "ScaledJoystickVals");
    ros::NodeHandle n_;
    ScaledValPub = n_.advertise<ros_control_boilerplate::JoystickState>("ScaledJoystickVals", 1000);

    ros::spin();

    return 0;
}
