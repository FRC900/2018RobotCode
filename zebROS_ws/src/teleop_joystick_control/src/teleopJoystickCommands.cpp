#include "teleop_joystick_control/teleopJoystickCommands.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/bool_srv.h"
#include "cstdlib"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/IntakeLiftAction.h"


/*TODO list:
 *
 *
 *Note: arm, end_game_deploy, and intake will become services
 *
 *Press down to climb (lift goes to position)
 *
 */
//using namespace message_filters;

std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>> ac;

static double timeSecs = 0, lastTimeSecs = 0, directionUpLast = 0, YLast = 0, BLast = 0;
static std::string currentToggle = " ";
static std::string lastToggle = " ";
static double elevatorHeightBeforeX;
static double elevatorHeightBeforeY;
static bool hasCube;

static ros::Publisher JoystickRobotVel;
static ros::Publisher JoystickElevatorPos;
static ros::Publisher JoystickRumble;
static ros::Publisher EndGameDeploy;

static ros::ServiceClient ElevatorSrv;
static ros::ServiceClient ClampSrv;
static ros::ServiceClient IntakeSrv;

static double high_scale_config_x;
static double high_scale_config_y;
static double mid_scale_config_x;
static double mid_scale_config_y;
static double low_scale_config_x;
static double low_scale_config_y;
static double switch_config_x;
static double switch_config_y;
static double exchange_config_x;
static double exchange_config_y;
static double intake_config_x;
static double intake_config_y;
static double default_x;
static double default_y;
static double exchange_delay;
 

bool sendRobotZero = false;
bool sendArmZero = false;
// TODO : initialize values to meaningful defaults
bool ifCube = true;
double elevatorHeight;
static double elevatorPosX;
static double elevatorPosY;
static int i = 0;
static behaviors::IntakeLiftGoal elevatorGoal;
double navX_angle_ = M_PI/2;
int navX_index_ = -1;

ros::Subscriber navX_heading_;
ros::Subscriber elevator_odom;

void unToggle(void) {
    currentToggle = " "; 

    elevator_controller::ElevatorControl elevatorMsg;

    elevatorMsg.x = elevatorPosX;
    elevatorMsg.y = elevatorPosY;
    JoystickElevatorPos.publish(elevatorMsg);
}
void setHeight(void) {
    elevatorHeightBeforeX = elevatorPosX;
    elevatorHeightBeforeY = elevatorPosY;
}



void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState, const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {

    behaviors::IntakeLiftGoal goal;
    elevator_controller::ElevatorControl elevatorMsg;
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
    /*
    if(JoystickState->directionUpPress == true) {
    //TODO call auto climb file
    ROS_WARN("Auto climb");
    }
    if(JoystickState->directionUpRelease == true) {
        //TODO stop auto climb
        //publish a stop message?
    }
    */
	if(JoystickState->directionUpPress == true) {
		if(timeSecs - directionUpLast < 1.0) {
			std_msgs::Float64 msg;
			msg.data = 1.0;
			EndGameDeploy.publish(msg); //this will become a service
			ROS_WARN("SELF DESTURCT");
		}
		directionUpLast = timeSecs;
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

    //****************************************************\\
    /////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\
    //////////////////////// TOGGLES \\\\\\\\\\\\\\\\\\\\\\\


        lastToggle = currentToggle;

        elevator_controller::ElevatorControlS srvElevator;
        elevator_controller::bool_srv srvClamp;
        elevator_controller::Intake srvIntake;

        //**** MID LEVEL SCALE TOGGLE || Grab Cube ***//
        if(JoystickState->buttonXPress==true) {
            if(hasCube) {
                currentToggle = "X";
                if(lastToggle==" " || lastToggle == "XNoCube") {
                    setHeight();
                }
                if(currentToggle == lastToggle) {
                    unToggle();
                }
                else {
                    srvElevator.request.x = exchange_config_x;
                    srvElevator.request.y = exchange_config_y;
                    if(ElevatorSrv.call(srvElevator)) {
                        ROS_WARN("Toggled to mid level scale height");
                    }
                    else{
                        ROS_ERROR("Failed to toggle to mid level scale height");
                    }
                }
            }
            else {
                currentToggle = "XNoCube";
                if(lastToggle != currentToggle) {
                    srvClamp.request.data = true;
                    if(ClampSrv.call(srvClamp)) {
                        ROS_WARN("Clamped");
                    }
                    else {
                        ROS_ERROR("Failed to clamp");
                    }
                }
                else {
                    currentToggle = " ";
                    srvClamp.request.data = false;
                    if(ClampSrv.call(srvClamp)) {
                        ROS_WARN("UnClamped");
                    }
                    else {
                        ROS_ERROR("Failed to unclamp");
                    }
                }
            }
        }


        //**** Place cube if exchange config run intake out || intake cube ****//
        if(JoystickState->buttonAPress==true) {
            currentToggle = "A";
    
            if(hasCube) {
                if(lastToggle=="YDouble") {
                    srvIntake.request.spring_state = 2; //soft_in
                    if(IntakeSrv.call(srvIntake)) {
                        srvClamp.request.data = false;
                        if(ClampSrv.call(srvClamp)) {
                            srvIntake.request.power = -.8;
                            if(IntakeSrv.call(srvIntake)) {
                                ROS_WARN("Exchanged Cube");
                                ros::Duration(exchange_delay).sleep(); //TODO
                            }
                        }
                    }
                    else {
                        ROS_ERROR("Failed to exchange cube");
                    }
                }
                else {
                    srvClamp.request.data=false;
                    if(ClampSrv.call(srvClamp)) {
                        ROS_WARN("Placed cube");
                        goal.IntakeCube = false;
                        goal.GoToHeight = true;
                        goal.x = elevatorPosX - 1; //TODO
                        goal.y = elevatorPosY + 1; //TODO
                        ac->sendGoal(goal);
                        
                        if(ac->waitForResult(ros::Duration(1)) {
                            ROS_WARN("Moved arm out of the way");
                            srvElevator.request.x = intake_config_x;
                            srvElevator.request.y = intake_config_y;
                            if(ElevatorSrv.call(srvElevator)) {
                                ROS_WARN("Toggled to intake config");
                            }
                        }
                        else {
                            ROS_ERROR("Failed to move arm out of the way...");
                            ROS_ERROR("Manual correction required!");
                            ROS_ERROR("Manual correction required!");
                        }
                    }
                    else {
                        ROS_ERROR("Failed to place cube");
                    }
                }
            }
             

            if(lastToggle==" ") {
                unToggle();
            }
            if(currentToggle == lastToggle) {
                ac->cancelGoal();
                unToggle();
            }
            else {
                srvElevator.request.x = intake_config_x;
                srvElevator.request.y = intake_config_y;
                if(ElevatorSrv.call(srvElevator)) {
                    ROS_WARN("Toggled to intake config");
                }
                else{
                    ROS_ERROR("Failed to toggle to intake config");
                }
                
                goal.IntakeCube = true;
                ac->sendGoal(goal);
                ROS_WARN("Started intaking cube");

            }
        }

        if(timeSecs - YLast > .21 && timeSecs - YLast < .45) {
            currentToggle = "Yone";
            if(lastToggle==" ") {
                setHeight();
            }
            if(currentToggle == lastToggle) {
                unToggle();
            }
            else {
                srvElevator.request.x = switch_config_x;
                srvElevator.request.y = switch_config_y;
                if(ElevatorSrv.call(srvElevator)) {
                    ROS_WARN("Toggled to switch height");
                }
                else{
                    ROS_ERROR("Failed to toggle to switch height");
                }
                ROS_WARN("Toggled to switch height");
            }
            YLast = 0;
        }
        if(JoystickState->buttonYPress==true) {
            if(timeSecs - YLast < .2) {
                currentToggle = "Ydouble";
                if(lastToggle==" ") {
                    setHeight();
                }
                if(currentToggle == lastToggle) {
                    unToggle();
                }
                else {
                    srvElevator.request.x = exchange_config_x;
                    srvElevator.request.y = exchange_config_y;
                    if(ElevatorSrv.call(srvElevator)) {
                        ROS_WARN("Toggled to exchange config");
                    }
                    else{
                        ROS_ERROR("Failed to toggle to exchange config");
                    }
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
                setHeight();
            }
            if(currentToggle == lastToggle) {
                unToggle();
            }
            else {
                srvElevator.request.x = low_scale_config_x;
                srvElevator.request.y = low_scale_config_y;
                if(ElevatorSrv.call(srvElevator)) {
                    ROS_WARN("Toggled to low level scale");
                }
                else{
                    ROS_ERROR("Failed to toggle to low level scale");
                }
            }
            BLast = 0;
        }
        if(JoystickState->buttonBPress==true) {
            if(timeSecs - BLast < .2) {
                currentToggle = "doubleB";
                if(lastToggle==" ") {
                    setHeight();
                }
                if(currentToggle == lastToggle) {
                    unToggle();
                }
                else {
                    srvElevator.request.x = high_scale_config_x;
                    srvElevator.request.y = high_scale_config_y;
                    if(ElevatorSrv.call(srvElevator)) {
                        ROS_WARN("Toggled to high level scale");
                    }
                    else{
                        ROS_ERROR("Failed to toggle to high level scale");
                    }
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
    //talon_controllers::CloseLoopControllerMsg arm;

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
    
    if(fabs(vel.linear.x) != 0.0 || fabs(vel.linear.y) != 0.0 || fabs(vel.angular.z) != 0.0 || sendRobotZero) {
        JoystickRobotVel.publish(vel);
        sendRobotZero = true;
        if(fabs(vel.linear.x) == 0.0 || fabs(vel.linear.y) == 0.0 || fabs(vel.angular.z) == 0.0) {
            i+=1;
            if(i>10){
                sendRobotZero = false;
                i=0;
            }
        }
    }
    if(rightStickX != 0 && rightStickY != 0) {
        elevatorPosX += (timeSecs-lastTimeSecs)*rightStickX; //Access current elevator position to fix code allowing out of bounds travel
        elevatorPosY += (timeSecs-lastTimeSecs)*rightStickY; //Access current elevator position to fix code allowing out of bounds travel
        elevatorMsg.x = elevatorPosX;
        elevatorMsg.y = elevatorPosY;
        JoystickElevatorPos.publish(elevatorMsg);
    }

        //TODO BUMPERS FOR SLOW MODE
        //ROS_WARN("leftStickX: %f", leftStickX);
        //ROS_WARN("leftStickY: %f", leftStickY);
    //TODO BUMPERS FOR SLOW MODE
    //TODO rotate left
    //TODO rotate right
	//ROS_WARN("be afraid");
    lastTimeSecs = timeSecs;
}

void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
   elevatorPosX =  msg->x;
   elevatorPosY =  msg->y;
}
/*
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
*/
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
    ros::init(argc, argv, "Joystick_controller");
    ros::NodeHandle n;

    ros::NodeHandle n_params(n, "teleop_params");
    
    n_params.getParam("high_scale_config_x", high_scale_config_x);
    n_params.getParam("high_scale_config_y", high_scale_config_y);
    n_params.getParam("mid_scale_config_x", mid_scale_config_x);
    n_params.getParam("mid_scale_config_y", mid_scale_config_y);
    n_params.getParam("low_scale_config_x", low_scale_config_x);
    n_params.getParam("low_scale_config_y", low_scale_config_y);
    n_params.getParam("switch_config_x", switch_config_x);
    n_params.getParam("switch_config_y", switch_config_y);
    n_params.getParam("exchange_config_x", exchange_config_x);
    n_params.getParam("exchange_config_y", exchange_config_y);
    n_params.getParam("intake_config_x", intake_config_x);
    n_params.getParam("intake_config_y", intake_config_y);
    n_params.getParam("default_x", default_x);
    n_params.getParam("default_y", default_y);
    n_params.getParam("exchange_delay", exchange_delay);


    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>>("auto_interpreter_server", true);

    JoystickRobotVel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    JoystickElevatorPos = n.advertise<elevator_controller::ElevatorControl>("/frcrobot/cmd_pos", 1);
    JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);
    EndGameDeploy = n.advertise<std_msgs::Float64>("/frcrobot/end_game_deploy_controller/command", 1);

    ElevatorSrv = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/cmd_posS");
    ClampSrv = n.serviceClient<elevator_controller::bool_srv>("/frcrobot/clamp");
    IntakeSrv = n.serviceClient<elevator_controller::Intake>("/frcrobot/intake");

    message_filters::Subscriber<ros_control_boilerplate::JoystickState> joystickSub(n, "scaled_joystick_vals", 5);
    message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> matchDataSub(n, "match_data", 5);

    navX_heading_ = n.subscribe("/frcrobot/navx_mxp", 1, &navXCallback);
    elevator_odom = n.subscribe("/frcrobot/odom", 1, &OdomCallback);

    ROS_WARN("joy_init");

    typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::JoystickState, ros_control_boilerplate::MatchSpecificData> JoystickSync;
    message_filters::Synchronizer<JoystickSync> sync(JoystickSync(5), joystickSub, matchDataSub);
    sync.registerCallback(boost::bind(&evaluateCommands, _1, _2));

    ac->waitForServer(); //Will wait for infinite time for server to start

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

