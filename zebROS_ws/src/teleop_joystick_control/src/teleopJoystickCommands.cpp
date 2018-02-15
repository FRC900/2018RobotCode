#include "teleop_joystick_control/teleopJoystickCommands.h"


//using namespace message_filters;
static double timeSecs = 0, lastTimeSecs = 0, directionRightLast = 0, YLast = 0, BLast = 0;
static std::string currentToggle = " ";
static std::string lastToggle = " ";
static double elevatorHeightBefore;
static ros::Publisher JoystickRobotVel;
static ros::Publisher JoystickArmVel;
static ros::Publisher JoystickRumble;

bool sendRobotZero = false;
bool sendArmZero = false;
// TODO : initialize values to meaningful defaults
bool ifCube = true;
double elevatorHeight;
static double armPos;

double navX_angle_ = M_PI/2;
int navX_index_ = -1;
ros::Subscriber navX_heading_;

void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState, const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {

    uint16_t leftRumble=0, rightRumble=0;
    double matchTimeRemaining = MatchData->matchTimeRemaining;
    timeSecs = ros::Time::now().toSec();
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
    /*
    if(RobotState->ifCube==true) {
        ifCube = true;
        rumbleTypeConverterPublish(0, 32767);
        ROS_WARN("I has cube");
    }
    else {
        ifCube = false;
        ROS_WARN("I has no cube");
    }
    elevatorHeight = RobotState->elevatorHeight;
*/

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
        ROS_WARN("Auto climb");
    }
    if(JoystickState->directionUpRelease == true) {
        //TODO stop auto climb
        //publish a stop message?
    }
    if(JoystickState->directionRightPress == true) {
        directionRightLast = timeSecs;
        timeSecs = ros::Time::now().toSec();
        if(timeSecs - directionRightLast < 1.0) {
            //TODO deploy ramp  or something
            //publish true to RampDeploy
            ROS_WARN("Deploy ramp");
        }
    }
    /*
    if(JoystickState->buttonBButton == true && ifCube==true) {
        //TODO auto scale
        //call auto scale file with a contained while loop that listens
        //on topic for stop command that is published to when
        //JoystickState->buttonBRelease == true
        ROS_WARN("Autoscale");
    }*/
    //else {

    //////////////////////// TOGGLES \\\\\\\\\\\\\\\\\\\\\\\\


        lastToggle = currentToggle;
        //exchange height toggle
        if(JoystickState->buttonXPress==true) {
            currentToggle = "X";
            if(lastToggle==" ") {
                elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                ROS_WARN("ElevatorHeightbefore set");
            }
            if(currentToggle == lastToggle) {
                currentToggle = " ";
                ROS_WARN("Untoggled");
                //TODO publish elevatorHeightBefore to ElevatorTarget or something
            }
            else {
                //TODO publish exchange height to ElevatorTarget or something
                ROS_WARN("Toggled to mid level scale height");
            }
        }

        //switch height toggle
        if(JoystickState->buttonAPress==true) {
            currentToggle = "A";
            if(lastToggle==" ") {
                elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                ROS_WARN("ElevatorHeightbefore set");
            }
            if(currentToggle == lastToggle) {
                currentToggle = " ";
                //TODO publish elevatorHeightBefore to ElevatorTarget or something
                ROS_WARN("Untoggled");
            }
            else {
                //TODO publish switch height to ElevatorTarget or something
                ROS_WARN("Toggled to intake config and start intake");
            }
        }

        if(timeSecs - YLast > .21 && timeSecs - YLast < .45) {
            currentToggle = "Yone";
            if(lastToggle==" ") {
                elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                ROS_WARN("ElevatorHeightbefore set");
            }
            if(currentToggle == lastToggle) {
                currentToggle = " ";
                //TODO publish elevatorHeightBefore to ElevatorTarget or something
                ROS_WARN("Untoggled");
            }
            else {
                //TODO publish switch height to ElevatorTarget or something
                ROS_WARN("Toggled to switch height");
            }
            YLast = 0;
        }
        if(JoystickState->buttonYPress==true) {
            if(timeSecs - YLast < .2) {
                currentToggle = "Ydouble";
                if(lastToggle==" ") {
                    elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                    ROS_WARN("ElevatorHeightbefore set");
                }
                if(currentToggle == lastToggle) {
                    currentToggle = " ";
                    //TODO publish elevatorHeightBefore to ElevatorTarget or something
                    ROS_WARN("Untoggled");
                }
                else {
                    //TODO publish switch height to ElevatorTarget or something
                    ROS_WARN("Toggled to exchange height");
                }
                YLast = 0;
            }
            else {
                YLast = timeSecs;
            }
        }
        if(timeSecs - BLast > .21 && timeSecs - BLast < .45) {
            currentToggle = "Bone";
            if(lastToggle==" ") {
                elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                ROS_WARN("ElevatorHeightbefore set");
            }
            if(currentToggle == lastToggle) {
                currentToggle = " ";
                //TODO publish elevatorHeightBefore to ElevatorTarget or something
                ROS_WARN("Untoggled");
            }
            else {
                //TODO publish switch height to ElevatorTarget or something
                ROS_WARN("Toggled to low level scale");
            }
            BLast = 0;
        }
        if(JoystickState->buttonBPress==true) {
            if(timeSecs - BLast < .2) {
                currentToggle = "doubleB";
                if(lastToggle==" ") {
                    elevatorHeightBefore = elevatorHeight; //TODO access elevator height
                    ROS_WARN("ElevatorHeightbefore set");
                }
                if(currentToggle == lastToggle) {
                    currentToggle = " ";
                    //TODO publish elevatorHeightBefore to ElevatorTarget or something
                    ROS_WARN("Untoggled");
                }
                else {
                    //TODO publish switch height to ElevatorTarget or something
                    ROS_WARN("Toggled to high level scale");
                }
                BLast = 0;

            }
            else {
                BLast = timeSecs;
            }
        }

   //}


///////////////////// Drivetrain and Elevator Control \\\\\\\\\\\\\\\\\\\


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
    
    if(fabs(vel.linear.x) != 0 || fabs(vel.linear.y) != 0 || fabs(vel.angular.z) != 0 || sendRobotZero) {
        JoystickRobotVel.publish(vel);
    }
    else {
        sendRobotZero = true;
    }
    armPos += .1*rightStickY; //Access current elevator position to fix code allowing out of bounds travel
    arm.command = armPos;

    JoystickArmVel.publish(arm);

        //TODO BUMPERS FOR SLOW MODE
        //ROS_WARN("leftStickX: %f", leftStickX);
        //ROS_WARN("leftStickY: %f", leftStickY);
    //TODO BUMPERS FOR SLOW MODE
    //TODO rotate left
    //TODO rotate right
	//ROS_WARN("be afraid");
}
void evaluateState(const teleop_joystick_control::RobotState::ConstPtr &RobotState) {
    if(RobotState->ifCube==true) {
        ifCube = true;
        rumbleTypeConverterPublish(0, 32767);
        ROS_WARN("I has cube");
    }
    else {
        ifCube = false;
        ROS_WARN("I has no cube");
    }
    elevatorHeight = RobotState->elevatorHeight;
}

/*
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
        allback(const sensor_msgs::Imu &navXState)

    }
    rumbleTypeConverterPublish(leftRumble, rightRumble);
}*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "scaled_joystick_state_subscriber");
    ros::NodeHandle n;

    JoystickRobotVel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    JoystickArmVel = n.advertise<talon_controllers::CloseLoopControllerMsg>("talon_linear_controller/command", 1);
    JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);

	// TODO : combine these into 1 callback with joystick val and robot
	// state synchronized by approximate message time.  See http://wiki.ros.org/message_filters#ApproximateTime_Policy
	// as well as the goal detection code for an example.  This will allow
	// the callback to get both a joystick value and the robot state
	// in one function.  Might want to combine match data as well?
    //message_filters::Subscriber<teleop_joystick_control::RobotState> robotStateSub(n, "RobotState", 1);
    message_filters::Subscriber<ros_control_boilerplate::JoystickState> joystickSub(n, "ScaledJoystickVals", 5);
    message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> matchDataSub(n, "match_data", 5);

    navX_heading_ = n.subscribe("/frcrobot/navx_mxp", 1, &navXCallback);

    ROS_WARN("joy_init");

    typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::JoystickState, ros_control_boilerplate::MatchSpecificData> JoystickSync;
    message_filters::Synchronizer<JoystickSync> sync(JoystickSync(5), joystickSub, matchDataSub);
    sync.registerCallback(boost::bind(&evaluateCommands, _1, _2));

    /*
    ros::Subscriber sub = n.subscribe("ScaledJoystickVals", 1, evaluateCommands);
    ros::Subscriber MatchData = n.subscribe("match_data", 1, evaluateTime);
    //subscribe to robot state stuff for possession of cube and elevator height
    //added to global vars
    */
    ros::Subscriber sub2 = n.subscribe("RobotState", 1, evaluateState);

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
void navXCallback(const sensor_msgs::Imu &navXState)
{

	tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
	double roll;
	tf2::Matrix3x3(navQuat).getRPY(roll, roll, navX_angle_);
}

