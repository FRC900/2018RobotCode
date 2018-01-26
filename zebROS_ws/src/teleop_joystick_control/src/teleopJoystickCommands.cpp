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
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <string>

//using namespace message_filters;
static double timeSecs, lastTimeSecs;
static ros::Publisher JoystickRobotVel;
static ros::Publisher JoystickArmVel;
static ros::Publisher JoystickRumble;

// TODO : initialize values to meaningful defaults
bool ifCube = true;
double elevatorHeight;
static double armPos;

void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState, const teleop_joystick_control::RobotState::ConstPtr &RobotState, const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {
    char currentToggle = ' ';
    char lastToggle = ' ';
    double elevatorHeightBefore;

    uint16_t leftRumble=0, rightRumble=0;
    double matchTimeRemaining = MatchData->matchTimeRemaining;
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

    if(RobotState->ifCube==true) {
        ifCube = true;
        rumbleTypeConverterPublish(0, 32767);
        ROS_INFO("I has cube");
    }
    else {
        ifCube = false;
        ROS_INFO("I has no cube");
    }
    elevatorHeight = RobotState->elevatorHeight;


    //Joystick Rumble
    if(matchTimeRemaining < 61 && matchTimeRemaining > 59) { 
        leftRumble = 65535;
    }
    else if(matchTimeRemaining < 31 && matchTimeRemaining > 29) {
        rightRumble = 65535;
    }
    else if(matchTimeRemaining <17 && matchTimeRemaining > 14) {
        leftRumble = 65535;
        rightRumble = 65535;
    }
    rumbleTypeConverterPublish(leftRumble, rightRumble);
    



    //Joystick Button Press parse
    if(JoystickState->directionUpPress == true) {
        //TODO call auto climb file
        ROS_INFO("Auto climb");
    }
    if(JoystickState->directionUpRelease == true) {
        //TODO stop auto climb
        //publish a stop message?
    }
    if(JoystickState->directionRightPress == true) {
        lastTimeSecs = timeSecs;
        timeSecs = ros::Time::now().toSec();
        if(timeSecs - lastTimeSecs< 1.0) {
            //TODO deploy ramp  or something
            //publish true to RampDeploy
            ROS_INFO("Deploy ramp");
        }
    }
    if(JoystickState->buttonBButton == true && ifCube==true) {
        //TODO auto scale
        //call auto scale file with a contained while loop that listens
        //on topic for stop command that is published to when
        //JoystickState->buttonBRelease == true
        ROS_INFO("Autoscale");
    }
    else {
        if(JoystickState->buttonBPress == true) {    
            if(ifCube == false) {
                //TODO go to intake height
                //publish half height to ElevatorTarget or something
                ROS_INFO("go to intake height");
            }
        }

        lastToggle = currentToggle;
        //exchange height toggle
        if(JoystickState->buttonXPress==true) {
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
        if(JoystickState->buttonYPress==true) {
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


    //Publish drivetrain messages and elevator/pivot
    geometry_msgs::Twist vel;
    talon_controllers::CloseLoopControllerMsg arm;
    double rightStickX = JoystickState->rightStickX;
    double rightStickY = JoystickState->rightStickY;

    Eigen::Vector2d joyVector;

    joyVector[1] = -JoystickState->leftStickX; //intentionally flipped
    joyVector[0] = JoystickState->leftStickY;
    Eigen::Rotation2Dd r(M_PI / 2 - navX_angle_);
    Eigen::Vector2d rotatedJoyVector = r.toRotationMatrix() * joyVector;
    vel.linear.x = rotatedJoyVector[0];
    vel.linear.y = rotatedJoyVector[1];
    vel.linear.z = 0;

    vel.angular.z = JoystickState->leftTrigger - JoystickState->rightTrigger;
    vel.angular.x = 0;
    vel.angular.y = 0;
    
 
    armPos += .1*rightStickY; //Access current elevator position to fix code allowing out of bounds travel
    arm.command = armPos;

    JoystickRobotVel.publish(vel);
    JoystickArmVel.publish(arm);
    ros::spinOnce();    


            
        //TODO BUMPERS FOR SLOW MODE
        //ROS_INFO("leftStickX: %f", leftStickX);
        //ROS_INFO("leftStickY: %f", leftStickY);
    //TODO BUMPERS FOR SLOW MODE
    //TODO rotate left
    //TODO rotate right
	//ROS_WARN("be afraid");            
}
/*
void evaluateState(const teleop_joystick_control::RobotState::ConstPtr &RobotState) {
    if(RobotState->ifCube==true) {
        ifCube = true;
        rumbleTypeConverterPublish(0, 32767);
        ROS_INFO("I has cube");
    }
    else {
        ifCube = false;
        ROS_INFO("I has no cube");
    }
    elevatorHeight = RobotState->elevatorHeight;
}

void evaluateTime(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {
    uint16_t leftRumble=0, rightRumble=0;
    double matchTimeRemaining = MatchData->matchTimeRemaining;
	// TODO : make these a set of else if blocks?
    if(matchTimeRemaining < 61 && matchTimeRemaining > 59) { 
        leftRumble = 65535;
    }
    else if(matchTimeRemaining < 31 && matchTimeRemaining > 29) {
        rightRumble = 65535;
    }
    else if(matchTimeRemaining <17 && matchTimeRemaining > 14) {
        leftRumble = 65535;
        rightRumble = 65535;
    }
    rumbleTypeConverterPublish(leftRumble, rightRumble);
}*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "scaled_joystick_state_subscriber");
    ros::NodeHandle n;
	// TODO : combine these into 1 callback with joystick val and robot 
	// state synchronized by approximate message time.  See http://wiki.ros.org/message_filters#ApproximateTime_Policy
	// as well as the goal detection code for an example.  This will allow
	// the callback to get both a joystick value and the robot state
	// in one function.  Might want to combine match data as well?
    message_filters::Subscriber<ros_control_boilerplate::JoystickState> joystickSub(n, "ScaledJoystickVals", 1);
    message_filters::Subscriber<teleop_joystick_control::RobotState> robotStateSub(n, "RobotState", 1);
    message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> matchDataSub(n, "match_data", 1);
    
    navX_heading_ = n.subscribe("/frcrobot/joint_states", 1, &navXCallback);


    typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::JoystickState, teleop_joystick_control::RobotState, ros_control_boilerplate::MatchSpecificData> JoystickSync;
    message_filters::Synchronizer<JoystickSync> sync(JoystickSync(10), joystickSub, robotStateSub, matchDataSub);
    sync.registerCallback(boost::bind(&evaluateCommands, _1, _2, _3));
    
    /*
    ros::Subscriber sub = n.subscribe("ScaledJoystickVals", 1, evaluateCommands);
    //subscribe to robot state stuff for possession of cube and elevator height
    //added to global vars
    ros::Subscriber sub2 = n.subscribe("RobotState", 1, evaluateState);
    ros::Subscriber MatchData = n.subscribe("match_data", 1, evaluateTime);
    */
    JoystickRobotVel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    JoystickArmVel = n.advertise<talon_controllers::CloseLoopControllerMsg>("talon_linear_controller/command", 1);
    JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);

    ros::spin();

    return 0;
}
void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble) { 
    unsigned int rumble = ((leftRumble & 0xFFFF) << 16) | (rightRumble & 0xFFFF); 
    double rumble_val;
    rumble_val = *((double*)(&rumble));
    std_msgs::Float64 rumbleMsg;
    rumbleMsg.data = rumble_val;
    JoystickRumble.publish(rumbleMsg);
}
/*
void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble) {
    unsigned int rumble = ((leftRumble & 0xFFFF) << 16) | (rightRumble & 0xFFFF);
    double rumble_val;
    rumble_val = *((double*)(&rumble));
    std_msgs::Float64 rumbleMsg;
    rumbleMsg.data = rumble_val;
    JoystickRumble.publish(rumbleMsg);
}
*/
void navXCallback(const sensor_msgs::JointState &navXState)
{
	if(navX_index_ < 0)
	{
		for (size_t i = 0; i < navXState.name.size(); i++)
		{

			//ROS_WARN("gets calling");
			if(navXState.name[i] == "navX_0")
			{
				navX_index_ = i;
				ROS_WARN("navX not found");
				break;
			}

		}
	}

	if(navX_index_ > -1)
	{
		navX_angle_ = navXState.position[navX_index_];
		//ROS_INFO_STREAM("Works: " << navX_angle_);

	}

}

