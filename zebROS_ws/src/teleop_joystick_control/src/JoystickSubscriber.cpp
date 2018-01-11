#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"

#include <math.h>

#define DEAD .1

double deadzone(double val) {
    if(fabs(val)<=DEAD) {
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


    
    double scaledLeftStickX = pow(deadzone(leftStickX), 3);
    double scaledLeftStickY = pow(deadzone(leftStickY), 3);

    double scaledRightStickX = pow(deadzone(rightStickX), 3);
    double scaledRightStickY = pow(deadzone(rightStickY), 3);
    //ROS_INFO("scaledLetStickX: %f scaledLeftStickY: %f\n", scaledLeftStickX, scaledLeftStickY);
    int i = 1;
    char* tmp = "a";
    ros::init(i, &tmp, "ScaledJoystickVals");
    ros::NodeHandle n;
    ros::Publisher ScaledValPub =
    n.advertise<ros_control_boilerplate::JoystickState>("ScaledJoystickVals",
    1000);
    ros::Rate loop_rate(10);

    while(ros::ok()) {
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


        ScaledValPub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "joystick_state_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joystick_states", 1000, joystick);
    ros::spin();

    return 0;
}
