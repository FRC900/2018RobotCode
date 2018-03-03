#include <atomic>
#include "teleop_joystick_control/teleopJoystickCommands.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/bool_srv.h"
#include "elevator_controller/Blank.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/IntakeLiftAction.h"
#include "talon_swerve_drive_controller/Blank.h"

/*TODO list:
 *
 *
 *Note: arm, end_game_deploy, and intake will become services
 *
 *Press down to climb (lift goes to position)
 *
 */
//using namespace message_filters;
static double dead_zone=.2, slow_mode=.33, max_speed=3.3, max_rot=7.65/3, joystick_scale=3;
double dead_zone_check(double val) {
    if(fabs(val)<=dead_zone) {
        return 0;
    }
    return val;
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>> ac;

static double timeSecs = 0, lastTimeSecs = 0, directionUpLast = 0, YLast = 0, BLast = 0, ALast = 0,ADoubleStart = 0;
static bool run_out = false;
static std::string currentToggle = " ";
static std::string lastToggle = " ";
static double elevatorPosBeforeX;
static double elevatorPosBeforeY;
static bool elevatorUpOrDownBefore;
std::atomic<bool> hasCube;

static ros::Publisher JoystickRobotVel;
static ros::Publisher JoystickTestVel;
static ros::Publisher JoystickElevatorPos;
static ros::Publisher JoystickRumble;
static ros::ServiceClient EndGameDeploy;

static ros::ServiceClient ElevatorSrv;
static ros::ServiceClient ClampSrv;
static ros::ServiceClient IntakeSrv;
ros::ServiceClient brake_srv;


static double high_scale_config_x;
static double high_scale_config_y;
static double high_scale_config_up_or_down;
static double mid_scale_config_x;
static double mid_scale_config_y;
static double mid_scale_config_up_or_down;
static double low_scale_config_x;
static double low_scale_config_y;
static double low_scale_config_up_or_down;
static double switch_config_x;
static double switch_config_y;
static double switch_config_up_or_down;
static double exchange_config_x;
static double exchange_config_y;
static double exchange_config_up_or_down;
static double intake_ready_to_drop_x;
static double intake_ready_to_drop_y;
static double intake_ready_to_drop_up_or_down;
static double intake_config_x;
static double intake_config_y;
static double intake_config_up_or_down;
static double climb;
static double default_x;
static double default_y;
static double default_up_or_down;
static double exchange_delay;

static bool intake_drop_up;

/*
static bool high_scale_up;
static bool mid_scale_up;
static bool low_scale_up;
static bool switch_up;
static bool exchange_up;
static bool intake_up;
static bool climb_up;
static bool default_up;
*/
 
std::atomic<bool> disable_arm_limits;
bool sendRobotZero = false;
bool sendArmZero = false;
// TODO : initialize values to meaningful defaults
bool ifCube = true;
//double elevatorHeight;
static std::atomic<double> elevatorPosX;
static std::atomic<double> elevatorPosY;
static std::atomic<bool> elevatorUpOrDown;
static int i = 0;
static behaviors::IntakeLiftGoal elevatorGoal;
std::atomic<double> navX_angle_;

ros::Subscriber navX_heading_;
ros::Subscriber match_data;
ros::Subscriber cube_state_;
ros::Subscriber joystick_sub;
ros::Subscriber elevator_odom;
ros::Subscriber disable_arm_limits_sub;

void unToggle(void) {
    currentToggle = " "; 
    elevator_controller::ElevatorControlS srvElevator;

    srvElevator.request.x = elevatorPosBeforeX;
    srvElevator.request.y = elevatorPosBeforeY;
    srvElevator.request.up_or_down = elevatorUpOrDownBefore;
    srvElevator.request.override_pos_limits = disable_arm_limits;
    ElevatorSrv.call(srvElevator);
}
void setHeight(void) {
    elevatorPosBeforeX = elevatorPosX;
    elevatorPosBeforeY = elevatorPosY;
    elevatorUpOrDownBefore = elevatorUpOrDown;
}

void match_data_callback(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {
    uint16_t leftRumble=0, rightRumble=0;
    //Joystick Rumble
    double matchTimeRemaining = MatchData->matchTimeRemaining;

    if((matchTimeRemaining < 91 && matchTimeRemaining > 90.8) || ( matchTimeRemaining < 90.7 && matchTimeRemaining > 90.5) || (matchTimeRemaining < 90.4 && matchTimeRemaining > 90.2) || ( matchTimeRemaining < 90.1 && matchTimeRemaining > 89.9) ) {
        leftRumble = 65535;
        rightRumble = 65535;
    }
    if((matchTimeRemaining < 61 && matchTimeRemaining > 60.8) || ( matchTimeRemaining < 60.7 && matchTimeRemaining > 60.5) || (matchTimeRemaining < 60.4 && matchTimeRemaining > 60.2)) {
        leftRumble = 65535;
        rightRumble = 65535;
    }
    if((matchTimeRemaining < 31 && matchTimeRemaining > 30.8) || ( matchTimeRemaining < 30.7 && matchTimeRemaining > 30.5)) {
        leftRumble = 65535;
        rightRumble = 65535;
    }
    if((matchTimeRemaining < 11 && matchTimeRemaining > 10.8)) {
        leftRumble = 65535;
        rightRumble = 65535;
    }
    rumbleTypeConverterPublish(leftRumble, rightRumble);

}

void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState) {

    behaviors::IntakeLiftGoal goal;
    elevator_controller::ElevatorControl elevatorMsg;
    //double matchTimeRemaining = MatchData->matchTimeRemaining;
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
			elevator_controller::Blank msg; //TODO
			msg.request.nothing_here = true;
			EndGameDeploy.call(msg);
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

        if(JoystickState->directionDownPress == true) {
            srvElevator.request.y = climb;
            srvElevator.request.x = elevatorPosX;
            srvElevator.request.up_or_down = elevatorUpOrDown;
    	    srvElevator.request.override_pos_limits = disable_arm_limits;
            ElevatorSrv.call(srvElevator);
            ROS_WARN("Climb config");
        }
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
                    srvElevator.request.x = mid_scale_config_x;
                    srvElevator.request.y = mid_scale_config_y;
                    srvElevator.request.up_or_down = mid_scale_config_up_or_down;
    	    	    srvElevator.request.override_pos_limits = disable_arm_limits;
                    if(ElevatorSrv.call(srvElevator)) {
                        ROS_WARN("Toggled to mid level scale height");
                    }
                    else{
                        ROS_ERROR("Failed to toggle to mid level scale height");
                    }
                }
            }
            else {
				static bool clamped;
                if(clamped == false) {
                    srvClamp.request.data = true;
                    if(ClampSrv.call(srvClamp)) {
                        ROS_WARN("Clamped");
                        clamped = true;
                    }
                    else {
                        ROS_ERROR("Failed to clamp");
                    }
                }
                else {
                    srvClamp.request.data = false;
                    if(ClampSrv.call(srvClamp)) {
                        ROS_WARN("UnClamped");
                        clamped = false;
                    }
                    else {
                        ROS_ERROR("Failed to unclamp");
                    }
                }
            }
        }


        //**** Place cube if exchange config run intake out || intake cube ****//
        if(JoystickState->buttonAPress) {
            if(hasCube) {
                if(timeSecs - ALast > .31 && timeSecs - ALast < .5) {
            
                    if(hasCube) {
                        if(lastToggle=="YDouble") {
                            goal.IntakeCube = false;
                            goal.MoveArmAway = false;
                            goal.GoToHeight = true;
                            goal.x = intake_ready_to_drop_x;
                            goal.y = intake_ready_to_drop_y;
                            goal.up_or_down = intake_drop_up;
                            ac->sendGoal(goal);
                            ac->waitForResult(ros::Duration(1));
                            
                            goal.x = intake_config_x;
                            goal.y = intake_config_y;
                            goal.up_or_down = intake_config_up_or_down;
                            ac->sendGoal(goal);
                            if(ac->waitForResult(ros::Duration(15))) {
                                srvIntake.request.power = 0;
                                srvIntake.request.spring_state = 2; //soft_in
                                IntakeSrv.call(srvIntake);

                                srvClamp.request.data = false;
                                ClampSrv.call(srvClamp);

                                srvIntake.request.power = -0.8;
                                srvIntake.request.spring_state = 2; //soft_in
                                IntakeSrv.call(srvIntake);
                                ros::Duration(.5).sleep();
                                srvElevator.request.x = elevatorPosX;
                                srvElevator.request.y = intake_ready_to_drop_y;
                                ElevatorSrv.call(srvElevator);
                            }
                        }
                        else {
                            srvClamp.request.data=false;
                            behaviors::IntakeLiftGoal goal;
                            if(ClampSrv.call(srvClamp)) {
                                ROS_WARN("Placed cube");
                                goal.IntakeCube = false;
                                goal.GoToHeight = false;
                                goal.MoveArmAway = true;
                                goal.x = intake_config_x;
                                goal.y = intake_config_y;
                                goal.up_or_down = intake_config_up_or_down;
                                /*
                                goal.IntakeCube = false;
                                goal.GoToHeight = true;
                                goal.x = elevatorPosX - 1; //TODO
                                goal.y = elevatorPosY + 1; //TODO
                                ac->sendGoal(goal);
                                
                                if(ac->waitForResult(ros::Duration(1))) {
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
                                */
                            }
                            else {
                                ROS_ERROR("Failed to place cube");
                            }
                        }
                        ALast = 0;
                    }
                }
                if(JoystickState->buttonAPress==true) {
                    if(hasCube) {
                        if(timeSecs - ALast < .3) {
                            ADoubleStart = timeSecs;
							run_out = true;
                            srvIntake.request.power = -1;
                            srvIntake.request.spring_state = 2; //soft_in
                            srvIntake.request.up = false;
                            IntakeSrv.call(srvIntake);
                            ALast = 0;
                        }
                        else {
                            ALast = timeSecs;
                        }
                    }
                }
            }

            else{ 
                /* 
                if(lastToggle==" ") {
                    unToggle();
                }
                if(currentToggle == lastToggle) {
                    //ac->cancelGoal();
                    unToggle();
                }
                */
                //else {
                        goal.IntakeCube = false;
                        goal.GoToHeight = false;
                        goal.MoveArmAway = false;
                        goal.IntakeCubeNoLift = true;
                        ac->sendGoal(goal);
                    /* 
                    goal.IntakeCube = false;
                    goal.MoveArmAway = false;
                    goal.GoToHeight = true;
                    goal.x = intake_ready_to_drop_x;
                    goal.y = intake_ready_to_drop_y;
                    ac->sendGoal(goal);
                    ac->waitForResult(ros::Duration(1));
                    srvElevator.request.x = intake_config_x;
                    srvElevator.request.y = intake_config_y;
                    srvElevator.request.up_or_down = intake_config_up_or_down;
    	    	    srvElevator.request.override_pos_limits = disable_arm_limits;
                    if(ElevatorSrv.call(srvElevator)) {
                        ROS_WARN("Toggled to intake config");
                    }
                    else{
                        ROS_ERROR("Failed to toggle to intake config");
                    }
                    
                    goal.GoToHeight = false;
                    goal.IntakeCube = true;
                    ac->sendGoal(goal);
                    if(ac->waitForResult(ros::Duration(15))) {
                        srvClamp.request.data = true;
                        ClampSrv.call(srvClamp);
                        srvIntake.request.power = 0;
                        srvIntake.request.spring_state = 0; //out
                        IntakeSrv.call(srvIntake);
                        ros::Duration(.2).sleep();
                        srvElevator.request.x = elevatorPosX;
                        srvElevator.request.y = intake_ready_to_drop_y;
                        srvElevator.request.up_or_down = intake_ready_to_drop_up_or_down;
    	                srvElevator.request.override_pos_limits = disable_arm_limits;
                        ElevatorSrv.call(srvElevator);
                    }

                    ROS_WARN("Started intaking cube");
                    */

                //}
            }
        }
        if(run_out && (timeSecs > ADoubleStart + 2)) {
            srvIntake.request.power = 0;
            srvIntake.request.spring_state = 2; //soft_in
            srvIntake.request.up = false;
            IntakeSrv.call(srvIntake);
			run_out = false;
        }
        if(hasCube && !run_out) {
            srvIntake.request.power = .1;
            srvIntake.request.spring_state = 2; //soft_in
            srvIntake.request.up = false;
            IntakeSrv.call(srvIntake);
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
                srvElevator.request.up_or_down = switch_config_up_or_down;
    	        srvElevator.request.override_pos_limits = disable_arm_limits;
                if(ElevatorSrv.call(srvElevator)) {
                    ROS_WARN("Toggled to switch height");
                }
                else{
                    ROS_ERROR("Failed to toggle to switch height");
                }
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
                    srvElevator.request.up_or_down = exchange_config_up_or_down;
    	            srvElevator.request.override_pos_limits = disable_arm_limits;
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
        if(JoystickState->bumperRightPress==true) {
            currentToggle = "bumperRight";
            if(lastToggle==" ") {
                setHeight();
            }
            if(currentToggle == lastToggle) {
                unToggle();
            }
            else {
                srvElevator.request.x = default_x;
                srvElevator.request.y = default_y;
                srvElevator.request.up_or_down = default_up_or_down;
    	        srvElevator.request.override_pos_limits = disable_arm_limits;
                if(ElevatorSrv.call(srvElevator)) {
                    ROS_WARN("Toggled to default config");
                }
                else{
                    ROS_ERROR("Failed to toggle to default config");
                }
            }
        }
        if(JoystickState->buttonStartPress==true) {
            currentToggle = "start";
            srvIntake.request.power = 0;
            srvIntake.request.up = true;
            IntakeSrv.call(srvIntake);
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
                srvElevator.request.up_or_down = low_scale_config_up_or_down;
    	        srvElevator.request.override_pos_limits = disable_arm_limits;
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
                    srvElevator.request.up_or_down = high_scale_config_up_or_down;
    	            srvElevator.request.override_pos_limits = disable_arm_limits;
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
        if(JoystickState->buttonBackButton==true) {
            srvIntake.request.power = -.8;
            srvIntake.request.spring_state = 2; //soft_in
            IntakeSrv.call(srvIntake);
        }
        if(JoystickState->buttonBackRelease==true) {
            srvIntake.request.power = 0;
            IntakeSrv.call(srvIntake);
        }

   //}


///////////////////// Drivetrain and Elevator Control \\\\\\\\\\\\\\\\\\\


    //Publish drivetrain messages and elevator/pivot
    geometry_msgs::Twist vel;
    //talon_controllers::CloseLoopControllerMsg arm;
    double leftStickX = (pow(dead_zone_check(JoystickState->leftStickX), joystick_scale))*max_speed;
    double leftStickY = (-pow(dead_zone_check(JoystickState->leftStickY), joystick_scale))*max_speed;

    double rightStickX = (pow(dead_zone_check(JoystickState->rightStickX), joystick_scale));
    double rightStickY = (-pow(dead_zone_check(JoystickState->rightStickY), joystick_scale));

    Eigen::Vector2d joyVector;
    
    vel.angular.z = (JoystickState->leftTrigger - JoystickState->rightTrigger)*max_rot;
    vel.angular.x = 0;
    vel.angular.y = 0;
    
    if(JoystickState-> bumperLeftButton == true) {
        leftStickX *= slow_mode;
        leftStickY *= slow_mode;

        rightStickX *= slow_mode;
        rightStickY *= slow_mode;
        
	vel.angular.z *= slow_mode;
    }
    if((fabs(leftStickX) == 0.0 && fabs(leftStickY) == 0.0 && fabs(vel.angular.z) == 0.0) && !sendRobotZero) {
	talon_swerve_drive_controller::Blank blank;
        blank.request.nothing = true;
        brake_srv.call(blank);
        sendRobotZero = true;
    }
    else if(fabs(leftStickX) != 0.0 || fabs(leftStickY) != 0.0 || fabs(vel.angular.z) != 0.0)
    {


    joyVector[0] = leftStickX; //intentionally flipped
    joyVector[1] = -leftStickY;
    Eigen::Rotation2Dd r(-navX_angle_ - M_PI/2);
    Eigen::Vector2d rotatedJoyVector = r.toRotationMatrix() * joyVector;
	vel.linear.x = rotatedJoyVector[1];
	vel.linear.y = rotatedJoyVector[0];
        JoystickRobotVel.publish(vel);
	std_msgs::Header test_header;
	test_header.stamp = JoystickState -> header.stamp;
	JoystickTestVel.publish(test_header);
	sendRobotZero = false;
    }
    if(rightStickX != 0 && rightStickY != 0) {
        elevatorPosX = elevatorPosX + (timeSecs-lastTimeSecs)*rightStickX; //Access current elevator position to fix code allowing out of bounds travel
        elevatorPosY = elevatorPosY + (timeSecs-lastTimeSecs)*rightStickY; //Access current elevator position to fix code allowing out of bounds travel
        elevatorMsg.x = elevatorPosX;
        elevatorMsg.y = elevatorPosY;
	elevatorMsg.up_or_down = elevatorUpOrDown;
	elevatorMsg.override_pos_limits = disable_arm_limits; 
        JoystickElevatorPos.publish(elevatorMsg);
    }

    lastTimeSecs = timeSecs;
}

void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
   elevatorPosX =  msg->x;
   elevatorPosY =  msg->y;
   elevatorUpOrDown =msg->up_or_down;
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
    
    if (!n_params.getParam("high_scale_config_x", high_scale_config_x))
		ROS_ERROR("Could not read high_scale_config_x");
    if (!n_params.getParam("high_scale_config_y", high_scale_config_y))
		ROS_ERROR("Could not read high_scale_config_y");
    if (!n_params.getParam("high_scale_config_up_or_down", high_scale_config_up_or_down))
		ROS_ERROR("Could not read high_scale_config_up_or_down");
    if (!n_params.getParam("mid_scale_config_x", mid_scale_config_x))
		ROS_ERROR("Could not read mid_scale_config_x");
    if (!n_params.getParam("mid_scale_config_y", mid_scale_config_y))
		ROS_ERROR("Could not read mid_scale_config_y");
    if (!n_params.getParam("mid_scale_config_up_or_down", mid_scale_config_up_or_down))
		ROS_ERROR("Could not read mid_scale_config_up_or_down");
    if (!n_params.getParam("low_scale_config_x", low_scale_config_x))
		ROS_ERROR("Could not read low_scale_config_x");
    if (!n_params.getParam("low_scale_config_y", low_scale_config_y))
		ROS_ERROR("Could not read low_scale_config_y");
    if (!n_params.getParam("low_scale_config_up_or_down", low_scale_config_up_or_down))
		ROS_ERROR("Could not read low_scale_config_up_or_down");
    if (!n_params.getParam("switch_config_x", switch_config_x))
		ROS_ERROR("Could not read switch_config_x");
    if (!n_params.getParam("switch_config_y", switch_config_y))
		ROS_ERROR("Could not read switch_config_y");
    if (!n_params.getParam("switch_config_up_or_down", switch_config_up_or_down))
		ROS_ERROR("Could not read switch_config_up_or_down");
    if (!n_params.getParam("exchange_config_x", exchange_config_x))
		ROS_ERROR("Could not read exchange_config_x");
    if (!n_params.getParam("exchange_config_y", exchange_config_y))
		ROS_ERROR("Could not read exchange_config_y");
    if (!n_params.getParam("exchange_config_up_or_down", exchange_config_up_or_down))
		ROS_ERROR("Could not read exchange_config_up_or_down");
    if (!n_params.getParam("intake_ready_to_drop_x", intake_ready_to_drop_x))
		ROS_ERROR("Could not read intake_ready_to_drop_x");
    if (!n_params.getParam("intake_ready_to_drop_y", intake_ready_to_drop_y))
		ROS_ERROR("Could not read intake_ready_to_drop_y");
    if (!n_params.getParam("intake_ready_to_drop_up_or_down", intake_ready_to_drop_up_or_down))
		ROS_ERROR("Could not read intake_ready_to_drop_up_or_down");
    if (!n_params.getParam("intake_config_x", intake_config_x))
		ROS_ERROR("Could not read intake_config_x");
    if (!n_params.getParam("intake_config_y", intake_config_y))
		ROS_ERROR("Could not read intake_config_y");
    if (!n_params.getParam("intake_config_up_or_down", intake_config_up_or_down))
		ROS_ERROR("Could not read intake_config_up_or_down");
    if (!n_params.getParam("climb", climb))
		ROS_ERROR("Could not read climb");
    if (!n_params.getParam("default_x", default_x))
		ROS_ERROR("Could not read default_x");
    if (!n_params.getParam("default_y", default_y))
		ROS_ERROR("Could not read default_y");
    if (!n_params.getParam("default_up_or_down", default_up_or_down))
		ROS_ERROR("Could not read default_up_or_down");
    if (!n_params.getParam("exchange_delay", exchange_delay))
		ROS_ERROR("Could not read exchange_delay");


    ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>>("auto_interpreter_server", true);

    JoystickRobotVel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    JoystickTestVel = n.advertise<std_msgs::Header>("test_header", 1);
    JoystickElevatorPos = n.advertise<elevator_controller::ElevatorControl>("/frcrobot/elevator_controller/cmd_pos", 1);
    JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);

    EndGameDeploy = n.serviceClient<elevator_controller::Blank>("/frcrobot/elevator_controller/end_game_deploy", 1);
    ElevatorSrv = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
    ClampSrv = n.serviceClient<elevator_controller::bool_srv>("/frcrobot/elevator_controller/clamp");
    IntakeSrv = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
    brake_srv = n.serviceClient<talon_swerve_drive_controller::Blank>("/frcrobot/talon_swerve_drive_controller/brake");

    //message_filters::Subscriber<ros_control_boilerplate::JoystickState> joystickSub(n, "scaled_joystick_vals", 5);
    //message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> matchDataSub(n, "match_data", 5);

    joystick_sub = n.subscribe("joystick_states", 1, &evaluateCommands);
    match_data = n.subscribe("match_data", 1, &match_data_callback);

    navX_heading_ = n.subscribe("/frcrobot/navx_mxp", 1, &navXCallback);
    elevator_odom = n.subscribe("/frcrobot/elevator_controller/odom", 1, &OdomCallback);
    cube_state_   = n.subscribe("/frcrobot/elevator_controller/cube_state", 1, &cubeCallback);
    disable_arm_limits_sub = n.subscribe("frcrobot/override_arm_limits", 1, &overrideCallback);

	disable_arm_limits = false;
	navX_angle_ = M_PI/2; 

    ROS_WARN("joy_init");

    //typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::JoystickState, ros_control_boilerplate::MatchSpecificData> JoystickSync;
    //message_filters::Synchronizer<JoystickSync> sync(JoystickSync(5), joystickSub, matchDataSub);
    //sync.registerCallback(boost::bind(&evaluateCommands, _1, _2));

    //ac->waitForServer(); //Will wait for infinite time for server to start

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
	double pitch;
	double yaw;
	tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);
	navX_angle_ = yaw;
}
void cubeCallback(const std_msgs::Bool &cube)
{
	hasCube = cube.data;
}
void overrideCallback(const std_msgs::Bool &override_lim)
{
	disable_arm_limits = override_lim.data;
}
