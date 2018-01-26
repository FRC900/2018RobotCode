#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"

#include <math.h>



static ros::Publisher ScaledValPub;
static double dead_zone=.1, slow_mode=.33, max_speed=3.3, max_rot=7.65, joystick_scale=3;
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

	// TODO : probably no reason to read these here, just assign them
	// to the appropriate fields in msa if they aren't used in
	// interim code

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
	// TODO : can just use -pow(), unless maybe we're going
	// to replae 0 with a constant some day?
    double scaledLeftStickY = (0-pow(dead_zoneCheck(leftStickY), joystick_scale))*max_speed;

    double scaledRightStickX = (0-pow(dead_zoneCheck(rightStickX),joystick_scale));
    double scaledRightStickY = (0-pow(dead_zoneCheck(rightStickY),joystick_scale));

    double scaledLeftTrigger = leftTrigger * max_rot;
    double scaledRightTrigger = rightTrigger * max_rot;
    if(bumperLeftButton == true) {
        scaledLeftStickX *= slow_mode;
        scaledLeftStickY *= slow_mode;
        
        scaledRightStickX *= slow_mode;
        scaledRightStickY *= slow_mode;
    
        scaledLeftTrigger *= slow_mode;
        scaledRightTrigger *= slow_mode;
    }
    //ROS_INFO("scaledLetStickX: %f scaledLeftStickY: %f\n", scaledLeftStickX, scaledLeftStickY);
    ros_control_boilerplate::JoystickState msa;

    msa.header = msg->header;
    msa.leftStickX = scaledLeftStickX;
    msa.leftStickY = scaledLeftStickY;

    msa.rightStickX = scaledRightStickX;
    msa.rightStickY = scaledRightStickY;

    msa.leftTrigger = scaledLeftTrigger;
    msa.rightTrigger = scaledRightTrigger;

    msa.buttonAButton = buttonAButton;
    msa.buttonAPress = buttonAPress;
    msa.buttonARelease = buttonARelease;

    msa.buttonBButton = buttonBButton;
    msa.buttonBPress = buttonBPress;
    msa.buttonBRelease = buttonBRelease;

    msa.buttonYButton = buttonYButton;
    msa.buttonYPress = buttonYPress;
    msa.buttonYRelease = buttonYRelease;

    msa.buttonXButton = buttonXButton;
    msa.buttonXPress = buttonXPress;
    msa.buttonXRelease = buttonXRelease;

    msa.bumperLeftButton = bumperLeftButton;
    msa.bumperLeftPress = bumperLeftPress;
    msa.bumperLeftRelease = bumperLeftRelease;

    msa.bumperRightButton = bumperRightButton;
    msa.bumperRightPress = bumperRightPress;
    msa.bumperRightRelease = bumperRightRelease;

    msa.buttonBackButton = buttonBackButton;
    msa.buttonBackPress = buttonBackPress;
    msa.buttonBackRelease = buttonBackRelease;

    msa.buttonStartButton = buttonStartButton;
    msa.buttonStartPress = buttonStartPress;
    msa.buttonStartRelease = buttonStartRelease;

    msa.stickLeftButton = stickLeftButton;
    msa.stickLeftPress = stickLeftPress;
    msa.stickLeftRelease = stickLeftRelease;

    msa.stickRightButton = stickRightButton;
    msa.stickRightPress = stickRightPress;
    msa.stickRightRelease = stickRightRelease;

   
    msa.directionUpButton = directionUpButton;
    msa.directionUpPress = directionUpPress;
    msa.directionUpRelease = directionUpRelease;

    msa.directionDownButton = directionDownButton;
    msa.directionDownPress = directionDownPress;
    msa.directionDownRelease = directionDownRelease;

    msa.directionLeftButton = directionLeftButton;
    msa.directionLeftPress = directionLeftPress;
    msa.directionLeftRelease = directionLeftRelease;

    msa.directionRightButton = directionRightButton;
    msa.directionRightPress = directionRightPress;
    msa.directionRightRelease = directionRightRelease;
    

    ScaledValPub.publish(msa);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "joystick_state_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joystick_states", 1, joystick);

    ScaledValPub = n.advertise<ros_control_boilerplate::JoystickState>("ScaledJoystickVals", 1);

    ros::spin();

    return 0;
}
