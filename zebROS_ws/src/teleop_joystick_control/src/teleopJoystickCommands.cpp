#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"
#include "teleop_joystick_control/RobotState.h"

#include <string>

bool ifCube;
double elevatorHeight;

void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &msg) {
    double timeSecs, lastTimeSecs;
    bool ifcube = false;
    char currentToggle = ' ';
    char lastToggle = ' ';
    double elevatorHeightBefore;
    /*
        map left joystick+bumpers+triggers into twist
        map right joystick into elevator/pivot position?
        autoscale to specific elevator height and pivot angle for now->->->
        
        press for auto climb height / pivot (CLIMB REGARDLESS OF CUBE)
        Double tap deploy ramp
        Auto scale (only with cube) (overrides drive train)
        Left joy - triggers
        Auto place - right Joy
        Toggles override each other and are turned off when the current toggle is pressed again - right joy press
    */
    if(msg->directionUpPress == true) {
        //TODO call auto climb file
        ROS_INFO("Auto climb");
    }
    if(msg->directionUpRelease == true) {
        //TODO stop auto climb
        //publish a stop message?
    }
    if(msg->directionRightPress == true) {
        lastTimeSecs = timeSecs;
        timeSecs = ros::Time::now().toSec();
        if(timeSecs - lastTimeSecs< 1.0) {
            //TODO deploy ramp  or something
            //publish true to RampDeploy
            ROS_INFO("Deploy ramp");
        }
    }
    if(msg->buttonBButton == true && ifCube==true) {
        //TODO auto scale
        //call auto scale file with a contained while loop that listens
        //on topic for stop command that is published to when
        //msg->buttonBRelease == true
        ROS_INFO("Autoscale");
    }
    else {
        if(msg->buttonBPress == true) {    
            if(ifCube == false) {
                //TODO go to intake height
                //publish half height to ElevatorTarget or something
                ROS_INFO("go to intake height");
            }
        }

        lastToggle = currentToggle;
        //exchange height toggle
        if(msg->buttonXPress==true) {
            currentToggle = 'X';
            if(lastToggle==' ') {
                elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                ROS_INFO("ElevatorHeightbefore set");
            }
            if(currentToggle == lastToggle) {
                currentToggle = ' ';
                ROS_INFO("Untoggled");
                //TODO publish elevatorHeightBefore to ElevatorTarget or something
            }
            else {
                //TODO publish exchange height to ElevatorTarget or something
                ROS_INFO("Toggled to exchange height");
            }
        }   

        //switch height toggle
        if(msg->buttonYPress==true) {
            currentToggle = 'Y';
            if(lastToggle==' ') {
                elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                ROS_INFO("ElevatorHeightbefore set");
            }
            if(currentToggle == lastToggle) {
                currentToggle = ' ';
                //TODO publish elevatorHeightBefore to ElevatorTarget or something
                ROS_INFO("Untoggled");
            }
            else {
                //TODO publish switch height to ElevatorTarget or something
                ROS_INFO("Toggled to switch height");
            }
        }

    }
    double leftStickX = msg->leftStickX;//TODO publish twist message for drivetrain and elevator/pivot
    double leftStickY = msg->leftStickY;
    //TODO BUMPERS FOR SLOW MODE
    ROS_INFO("leftStickX: %f", leftStickX);
    ROS_INFO("leftStickY: %f", leftStickY);
    double rightStickX = msg->rightStickX;//TODO publish twist message for drivetrain and elevator/pivot
    double rightStickY = msg->rightStickY;
    //TODO BUMPERS FOR SLOW MODE
    ROS_INFO("rightStickX: %f", rightStickX);
    ROS_INFO("rightStickY: %f", rightStickY);
    //TODO rotate left
    ROS_INFO("Rotate left: %f", msg->leftTrigger);
    //TODO rotate right
    ROS_INFO("Rotate right: %f", msg->rightTrigger);
    ROS_INFO("\n");
            
}

void evaluateState(const teleop_joystick_control::RobotState::ConstPtr &msg) {
    if(msg->ifCube==true) {
        ifCube = true;
        ROS_INFO("I has cube");
    }
    else {
        ifCube = false;
        ROS_INFO("I has no cube");
    }
    elevatorHeight = msg->elevatorHeight;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "scaled_joystick_state_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ScaledJoystickVals", 1000, evaluateCommands);
    //subscribe to robot state stuff for possession of cube and elevator height
    //added to global vars
    ros::init(argc, argv, "robot_state_subscriber");
    ros::NodeHandle n_;
    ros::Subscriber sub2 = n_.subscribe("RobotState", 1000, evaluateState);


    ros::spin();

    return 0;
}
