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
static double dead_zone=.2, slow_mode=.33, max_speed=3.3, max_rot=7.65/2, joystick_scale=3;
double dead_zone_check(double val) {
    if(fabs(val)<=dead_zone) {
        return 0;
    }
    return val;
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeLiftAction>> ac;

const double double_tap_zone = .3;
const double delay_after_single_tap = .31;
const double max_delay_after_single_tap = .45;

static double timeSecs = 0, lastTimeSecs = 0, directionUpLast = 0, YLast = 0, BLast = 0, ALast = 0,ADoubleStart = 0;
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
static bool high_scale_config_up_or_down;
static double mid_scale_config_x;
static double mid_scale_config_y;
static bool mid_scale_config_up_or_down;
static double low_scale_config_x;
static double low_scale_config_y;
static bool low_scale_config_up_or_down;
static double switch_config_x;
static double switch_config_y;
static bool switch_config_up_or_down;
static double exchange_config_x;
static double exchange_config_y;
static bool exchange_config_up_or_down;
static double intake_ready_to_drop_x;
static double intake_ready_to_drop_y;
static bool intake_ready_to_drop_up_or_down;
static double intake_config_x;
static double intake_config_y;
static bool intake_config_up_or_down;
static double climb;
static double default_x;
static double default_y;
static bool default_up_or_down;
static double exchange_delay;
static double move_out_pos_x;
static double move_out_pos_y;
static bool move_out_up_or_down;
static double move_out_down_y;
static bool ready_to_drop_check = false;

enum pos {high_scale, mid_scale, low_scale, switch_c, exchange, intake_ready_to_drop, intake, intake_low, climb_c, default_c, other};

static pos achieved_pos = other; //Not all neccesarily set after achieving, only if needed
static pos last_achieved_pos = other;
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
//double elevatorHeight;
static std::atomic<double> elevatorPosX;
static std::atomic<double> elevatorPosY;
static std::atomic<bool> elevatorUpOrDown;
static int i = 0;
static behaviors::IntakeLiftGoal elevatorGoal;
std::atomic<double> navX_angle_;
std::atomic<double> matchTimeRemaining;


ros::Subscriber navX_heading_;
ros::Subscriber match_data;
ros::Subscriber cube_state_;
ros::Subscriber joystick_sub;
ros::Subscriber elevator_odom;
ros::Subscriber disable_arm_limits_sub;

void unToggle(void) {
    currentToggle = " ";
    //TOGGLING BACK TO INTAKE/EXCHANGE CONFIG WON'T FULLY WORK 
    elevator_controller::ElevatorControlS srvElevator;

    srvElevator.request.x = elevatorPosBeforeX;
    srvElevator.request.y = elevatorPosBeforeY;
    srvElevator.request.up_or_down = elevatorUpOrDownBefore;
    srvElevator.request.override_pos_limits = disable_arm_limits;
    ElevatorSrv.call(srvElevator);
    achieved_pos = last_achieved_pos;
}
void setHeight(void) {
    elevatorPosBeforeX = elevatorPosX;
    elevatorPosBeforeY = elevatorPosY;
    elevatorUpOrDownBefore = elevatorUpOrDown;
    last_achieved_pos = achieved_pos;
}

void match_data_callback(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {
    uint16_t leftRumble=0, rightRumble=0;
    //Joystick Rumble
    matchTimeRemaining = MatchData->matchTimeRemaining;

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
//TODO: Overrides
//Only allow going to various buttons based on cube state
//Actually organize code so it can be debuged
// 
void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState) {
	/*std_msgs::Header first_header;
   	first_header.stamp = JoystickState -> header.stamp;
	first_header.seq = 0;
	JoystickTestVel.publish(first_header);*/

	elevator_controller::ElevatorControlS srvElevator;
	elevator_controller::bool_srv srvClamp;
	elevator_controller::Intake srvIntake;

	static bool run_out = false;
    behaviors::IntakeLiftGoal goal;
    goal.IntakeCube = false;
    goal.IntakeCubeNoLift = false;
    goal.GoToHeight = false;
    goal.MoveArmAway = false;
    goal.x = 0;
    goal.y = 0;
    goal.up_or_down = true;
    goal.override_pos_limits = false;
    elevator_controller::ElevatorControl elevatorMsg;
    timeSecs = ros::Time::now().toSec();

/*-----------------Up Double Press End Deploy------------------------------*/	

    if(JoystickState->directionUpPress == true && matchTimeRemaining < 30 ) {
		if(timeSecs - directionUpLast < 1.0) {
			elevator_controller::Blank msg; //TODO
			msg.request.nothing_here = true;
			EndGameDeploy.call(msg);
			ROS_WARN("SELF DESTURCT");
			achieved_pos = other;
		}
		directionUpLast = timeSecs;
    }
/*-----------------Down Press Climb to Correct height----------------------*/	
        if(JoystickState->directionDownPress == true) {
            srvElevator.request.y = climb;
            srvElevator.request.x = elevatorPosX;
            srvElevator.request.up_or_down = elevatorUpOrDown;
    	    srvElevator.request.override_pos_limits = disable_arm_limits;
            ElevatorSrv.call(srvElevator);
            ROS_WARN("Climb config");
	    achieved_pos = climb_c;
        }

    //****************************************************\\
    /////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\
    //////////////////////// TOGGLES \\\\\\\\\\\\\\\\\\\\\\\


        lastToggle = currentToggle;




/*-------------------------------------X------------------------------------*/	

	/*---------w/ Cube Single Press Toggle Mid Scale------*/	
        if(JoystickState->buttonXPress==true) {
            if(hasCube) {
                currentToggle = "X";
                if(lastToggle==" ") {
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
                achieved_pos = mid_scale;
                    if(ElevatorSrv.call(srvElevator)) {
                        ROS_WARN("Toggled to mid level scale height");
                    }
                    else{
                        ROS_ERROR("Failed to toggle to mid level scale height");
                    }
                }
            }
       /*------------No Cube Single Press Toggle Clamp------*/	
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

/*------------------------------------A------------------------------------*/	

	/*---------------------w/ Cube------------------------------*/	
	    static bool ready_to_spin_out_check = false;
	    static double place_start = 0;
	    static bool placed_delay_check = false;
            static bool go_to_intake_config;  
	    static bool manage_intaking; 
            if(hasCube) {
		/*----------------Single Press------------------*/	
                if(timeSecs - ALast > delay_after_single_tap && timeSecs - ALast < max_delay_after_single_tap) {  //wait .3 before assuming single press
		 /*-If in Exchange - Place and Run Intake Out-*/	 
                        if(lastToggle=="YDouble") {

                            ready_to_spin_out_check = true; //Kick off ready to drop
                        }
		 /*-Else - Place and Then Move Arm Back-*/
                        else {
                            srvClamp.request.data=false;
			    ClampSrv.call(srvClamp);
			     placed_delay_check = true; //Kick off placing
                             place_start = ros::Time::now().toSec();  
			  }
                        }
                        ALast = 0; //Remove flag
		 	/*Double Press - Just Go Out*/	 
                if(JoystickState->buttonAPress==true) {
                        if(timeSecs - ALast < double_tap_zone) {
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
	/*------------------No Cube - Single Press Intake-------------------*/	

	 else if(JoystickState->buttonAPress==true)
		{
                        /*ROS_WARN("intaking");
			goal.IntakeCube = false;   
                        goal.GoToHeight = false;
                        goal.MoveArmAway = false;
                        goal.IntakeCubeNoLift = true;
                        ac->sendGoal(goal);
			*/
			//TODO: eventually, disable above and enable below:
			//Further more, below and the associated stuff should be
			//Ported to an action lib (a new one)
			srvIntake.request.power = .8;
                        srvIntake.request.spring_state = 2; //soft_in
                        srvIntake.request.up = false;
                        IntakeSrv.call(srvIntake);
			go_to_intake_config = achieved_pos != intake;
			manage_intaking = true;

            }
	
	/*-If in Exchange - Place and Run Intake Out-*/	 

	static bool finish_spin_out_check = false;
	static double time_start_spin = 0;
	if(ready_to_spin_out_check)
	{
	    if(achieved_pos = intake)
	    {
		srvClamp.request.data = false;
		ClampSrv.call(srvClamp);

		srvIntake.request.power = -1;
		srvIntake.request.spring_state = 2; //soft_in
		srvIntake.request.up = false;
		IntakeSrv.call(srvIntake);
		finish_spin_out_check = true;		
                ready_to_spin_out_check = false;  
		time_start_spin = ros::Time::now().toSec();
	    }
	}
	
	/*-If just spun out, place and Run Intake Out-*/	 
	if(finish_spin_out_check && (timeSecs - time_start_spin) > 2.0)
	{
		srvIntake.request.power = 0;
                srvIntake.request.spring_state = 2; //soft_in
                srvIntake.request.up = false;
                IntakeSrv.call(srvIntake);
                finish_spin_out_check = false;
		
                srvElevator.request.x = default_x;
                srvElevator.request.y = default_y;
                srvElevator.request.up_or_down = default_up_or_down;
    	        srvElevator.request.override_pos_limits = disable_arm_limits;
                ElevatorSrv.call(srvElevator);
	    	achieved_pos = default_c;
	}
	static bool return_to_intake_from_low = false;
	static bool return_to_intake_from_high = false;
	static double return_to_intake_start = 0;
	/*If some time has passed since place, start arm move back*/	
	if(placed_delay_check && (timeSecs - place_start) > .5)
	{
		srvElevator.request.x = elevatorPosX  + move_out_pos_x; 
		srvElevator.request.y = elevatorPosY  + move_out_pos_y; 
                srvElevator.request.up_or_down = move_out_up_or_down;
                srvElevator.request.override_pos_limits = disable_arm_limits;
                ElevatorSrv.call(srvElevator);
		placed_delay_check = false;
                return_to_intake_from_low = achieved_pos == switch_c;
		return_to_intake_from_high = !return_to_intake_from_low;
		return_to_intake_start = timeSecs;
		achieved_pos = other;
	}
	if(return_to_intake_from_high && timeSecs - return_to_intake_start > 2)
	{
		/*
		TODO: Long term plan is to have intake return from high and intake return from low go up and then down and then drop after ~2 sec delay 
		ie:
		x:.1, y: cur_pos + 1, up_or_down=true -> get to Odom
		to
		x:.1, y: cur_pos + .05, up_or_down=false
				
		For right now we will just go back out and then call "go to intake config"
		*/
		return_to_intake_from_high = false;
		go_to_intake_config = true;
	
	}
	if(return_to_intake_from_low && timeSecs - return_to_intake_start > 2)
	{
		/*
		TODO: Long term plan is to have intake return from high and intake return from low go up and then down and then drop after ~2 sec delay 
		ie:
		x:.1, y: cur_pos + 1, up_or_down=true -> get to Odom
		to
		x:.1, y: cur_pos + .05, up_or_down=false
				
		For right now we will just go back out and then call "go to intake config"
		*/
		return_to_intake_from_low = false;
		go_to_intake_config = true;
	
	}
	/*When we get the cube, slow the intake and grip hard*/	
	static bool clamp_transfer =false;
	if(manage_intaking && hasCube)
	{
		manage_intaking = false;
		srvIntake.request.power = -0.1;
		srvIntake.request.spring_state = 3; //hard_in
		srvIntake.request.up = false;
		clamp_transfer = true;	

	}
	static bool go_to_default = false;
	static double clamp_grab_time  = 0;
	/*When we get the cube and the clamp is in position, grab the cube*/	
	if(clamp_transfer && achieved_pos == intake)
	{
		clamp_transfer =false;
		srvClamp.request.data = true;
		ClampSrv.call(srvClamp);	
		srvIntake.request.power = 0.0;
		srvIntake.request.spring_state = 1; //hard_out
		srvIntake.request.up = false;
		go_to_default = true;
		clamp_grab_time = timeSecs;		

	}
	/*Go to the normal resting point for a cube*/	
	if(go_to_default && timeSecs - clamp_grab_time > .75)
	{			
                srvElevator.request.x = default_x;
                srvElevator.request.y = default_y;
                srvElevator.request.up_or_down = default_up_or_down;
    	        srvElevator.request.override_pos_limits = disable_arm_limits;
	}	
	/*-below two ifs are exactly like above two ifs, except without clamp/arm/lift*/
        if(run_out && (timeSecs > ADoubleStart + 2)) {
            srvIntake.request.power = 0;
            srvIntake.request.spring_state = 2; //soft_in
            srvIntake.request.up = false;
            IntakeSrv.call(srvIntake);
            run_out = false;
        }
        if(hasCube && !run_out) {
            srvIntake.request.power = .1;
            srvIntake.request.spring_state = 3; //hard_in
            srvIntake.request.up = false;
            IntakeSrv.call(srvIntake);
        }

/*------------------------------------Y------------------------------------*/

    if(hasCube) //No need to go to switch/exchange without cube
    {
	
	/*-----------------Single Press - Switch ---------------------*/	
        
	if(timeSecs - YLast > delay_after_single_tap && timeSecs - YLast < max_delay_after_single_tap) {
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
	/*-----------------Double Press - Exchange ---------------------*/	

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
		    go_to_intake_config = achieved_pos!=intake;

		    }
                }
                YLast = 0;
            }
            else {
                YLast = timeSecs;
            }
        }	
	/*-Go to pos where intake can freely drop-*/	
	if(go_to_intake_config)
	{    
		goal.GoToHeight = true;
		goal.x = intake_ready_to_drop_x;
		goal.y = intake_ready_to_drop_y;
		goal.up_or_down = intake_ready_to_drop_up_or_down;
		goal.override_pos_limits = disable_arm_limits; 
		ac->sendGoal(goal);
		ready_to_drop_check  = true;
		go_to_intake_config = false;
		achieved_pos = other;
	
	}
	/*-----------------Drop Down---------------*/	
	static bool ready_to_drop_check = false;
	static bool check_intake = false;
	if(ready_to_drop_check)
        { 
	        if(ac->getState().isDone()) 
		{
		    goal.GoToHeight = true;
		    goal.x = intake_config_x;
		    goal.y = intake_config_y;
		    goal.up_or_down = intake_config_up_or_down;
		    goal.override_pos_limits = true; 
		    ac->sendGoal(goal);
		    ready_to_drop_check = false;	
		    check_intake = true;
                }
	}
	/*-----------------Finished---------------*/	
	if(check_intake)
	{
		if(ac->getState().isDone())
		{
			check_intake = false;
		    	achieved_pos = intake;
			//This needs to be checked so we can see whether or not we can push out a cube
		}

	}

	
/*----------------------------Right Bumper - Press Untoggle-----------------------------*/

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
		achieved_pos = default_c;
		if(ElevatorSrv.call(srvElevator)) {
                    ROS_WARN("Toggled to default config");
                }
                else{
                    ROS_ERROR("Failed to toggle to default config");
                }
            }
        }
/*---------------------Start Button - Press Bring Up Intake-------------------*/
        if(JoystickState->buttonStartPress==true) {
            currentToggle = "start";
            srvIntake.request.power = 0;
            srvIntake.request.up = true;
            srvIntake.request.spring_state = 2; //soft_in
            IntakeSrv.call(srvIntake);
        }
/*-----------------------------------------B------------------------------------------*/
    if(hasCube) //No need to go to switch/exchange without cube
    {
	
	/*--------------------Single Press - Low Scale ---------------------*/	
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
		achieved_pos = low_scale;
		
                if(ElevatorSrv.call(srvElevator)) {
                    ROS_WARN("Toggled to low level scale");
                }
                else{
                    ROS_ERROR("Failed to toggle to low level scale");
                }
            }
            BLast = 0;
        }
	/*--------------------Double Press - High Scale ---------------------*/	
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
		    achieved_pos = high_scale;
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
   }
/*------------------------Back Button - Hold Spin Out------------------------------------*/
        if(JoystickState->buttonBackButton==true) {
            srvIntake.request.power = -.8;
            srvIntake.request.up = false;
            srvIntake.request.spring_state = 2; //soft_in
            IntakeSrv.call(srvIntake);
        }
        if(JoystickState->buttonBackRelease==true) {
            srvIntake.request.power = 0;
            srvIntake.request.up = false;
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
	/*std_msgs::Header test_header;
	test_header.stamp = JoystickState -> header.stamp;
	test_header.seq = 1;
	JoystickTestVel.publish(test_header);*/
	sendRobotZero = false;
    }
    if(rightStickX != 0 && rightStickY != 0) {
        elevatorMsg.x = elevatorPosX + (timeSecs-lastTimeSecs)*rightStickX; //Access current elevator position to fix code allowing out of bounds travel
        elevatorMsg.y = elevatorPosY + (timeSecs-lastTimeSecs)*rightStickY; //Access current elevator position to fix code allowing out of bounds travel
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
    
    n_params.getParam("move_out_pos_x", move_out_pos_x);
    n_params.getParam("move_out_pos_y", move_out_pos_y);
    n_params.getParam("move_out_up_or_down", move_out_up_or_down);
    n_params.getParam("move_out_down_y", move_out_down_y);
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
    JoystickTestVel = n.advertise<std_msgs::Header>("test_header", 3);
    JoystickElevatorPos = n.advertise<elevator_controller::ElevatorControl>("/frcrobot/elevator_controller/cmd_pos", 1);
    JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);

    EndGameDeploy = n.serviceClient<elevator_controller::Blank>("/frcrobot/elevator_controller/end_game_deploy");
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
