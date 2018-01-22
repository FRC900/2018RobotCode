#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"
#include "teleop_joystick_control/RobotState.h"
#include "talon_controllers/CloseLoopControllerMsg.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "teleop_joystick_control/teleopJoystickCommands.h"
#include "ros/time.h"
#include <string>


bool ifCube;
double elevatorHeight;
static double timeSecs, lastTimeSecs;
static ros::Publisher JoystickRobotVel;
static ros::Publisher JoystickArmVel;
static ros::Publisher JoystickRumble;
static double armPos;
void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &msg) {
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
        ROS_INFO("%f, %f", lastTimeSecs, timeSecs);
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
    geometry_msgs::Twist vel;
    talon_controllers::CloseLoopControllerMsg arm;
    double leftStickX = msg->leftStickX;//TODO publish twist message for drivetrain and elevator/pivot
    double leftStickY = msg->leftStickY;

    double rightStickX = msg->rightStickX;//TODO publish twist message for drivetrain and elevator/pivot
    double rightStickY = msg->rightStickY;
    vel.linear.x = leftStickX;
    vel.linear.y = leftStickY;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;

    armPos += .1*rightStickY;
    arm.command = armPos;

    vel.angular.z = msg->leftTrigger - msg->rightTrigger;
    JoystickRobotVel.publish(vel);
    JoystickArmVel.publish(arm);
    ros::spinOnce();    


        //TODO BUMPERS FOR SLOW MODE
        //ROS_INFO("leftStickX: %f", leftStickX);
        //ROS_INFO("leftStickY: %f", leftStickY);
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
void evaluateTime(const ros_control_boilerplate::MatchSpecificData::ConstPtr &msg) {
    uint16_t leftRumble=0, rightRumble=0;
    double matchTimeRemaining = msg->matchTimeRemaining;
    if(matchTimeRemaining < 61 && matchTimeRemaining > 59) { 
        leftRumble = 65535;
    }
    if(matchTimeRemaining < 31 && matchTimeRemaining > 29) {
        rightRumble = 65535;
    }
    if(matchTimeRemaining <17 && matchTimeRemaining > 14) {
        leftRumble = 65535;
        rightRumble = 65535;
    }
    std_msgs::Float64 rumbleMsg;
    rumbleMsg.data = rumbleTypeConverter(leftRumble, rightRumble);
    JoystickRumble.publish(rumbleMsg);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "scaled_joystick_state_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ScaledJoystickVals", 1, evaluateCommands);
    //subscribe to robot state stuff for possession of cube and elevator height
    //added to global vars
    ros::Subscriber sub2 = n.subscribe("RobotState", 1, evaluateState);
    ros::Subscriber MatchData = n.subscribe("match_data", 1, evaluateTime);
    
    JoystickRobotVel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    JoystickArmVel = n.advertise<talon_controllers::CloseLoopControllerMsg>("talon_linear_controller/command", 1);
    JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);


    ros::spin();

    return 0;
}
double rumbleTypeConverter(uint16_t leftRumble, uint16_t rightRumble) { 
    int rumble = ((leftRumble & 0xFFFF) << 16) | (rightRumble & 0xFFFF); 
    double rumble_val;
    rumble_val = *((double*)(&rumble));
    return rumble_val;
}
