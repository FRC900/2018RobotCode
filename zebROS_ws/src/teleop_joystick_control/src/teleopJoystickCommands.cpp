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

static double time_secs = 0, last_time_secs = 0, directionUpLast = 0, YLast = 0, BLast = 0, ALast = 0,ADoubleStart = 0;
static std::string currentToggle = " ";
static std::string lastToggle = " ";
static double elevatorPosBeforeX;
static double elevatorPosBeforeY;
static bool elevator_up_or_downBefore;
std::atomic<bool> hasCube;

static ros::Publisher joystick_robot_vel;
static ros::Publisher joystick_test_vel;
static ros::Publisher joystick_elevator_pos;
static ros::Publisher joystick_rumble;
static ros::ServiceClient end_game_deploy;

static ros::ServiceClient elevator_service;
static ros::ServiceClient clamp_service;
static ros::ServiceClient intake_service;
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
static std::atomic<double> elevator_pos_x;
static std::atomic<double> elevator_pos_y;
static std::atomic<bool> elevator_up_or_down;
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
    elevator_controller::ElevatorControlS srv_elevator;

    srv_elevator.request.x = elevatorPosBeforeX;
    srv_elevator.request.y = elevatorPosBeforeY;
    srv_elevator.request.up_or_down = elevator_up_or_downBefore;
    srv_elevator.request.override_pos_limits = disable_arm_limits;
    elevator_service.call(srv_elevator);
    achieved_pos = last_achieved_pos;
}
void setHeight(void) {
    elevatorPosBeforeX = elevator_pos_x;
    elevatorPosBeforeY = elevator_pos_y;
    elevator_up_or_downBefore = elevator_up_or_down;
    last_achieved_pos = achieved_pos;
}

void match_data_callback(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData) {
    uint16_t left_rumble=0, right_rumble=0;
    //Joystick Rumble
    matchTimeRemaining = MatchData->matchTimeRemaining;

    if((matchTimeRemaining < 91 && matchTimeRemaining > 90.8) || ( matchTimeRemaining < 90.7 && matchTimeRemaining > 90.5) || (matchTimeRemaining < 90.4 && matchTimeRemaining > 90.2) || ( matchTimeRemaining < 90.1 && matchTimeRemaining > 89.9) ) {
        left_rumble = 65535;
        right_rumble = 65535;
    }
    if((matchTimeRemaining < 61 && matchTimeRemaining > 60.8) || ( matchTimeRemaining < 60.7 && matchTimeRemaining > 60.5) || (matchTimeRemaining < 60.4 && matchTimeRemaining > 60.2)) {
        left_rumble = 65535;
        right_rumble = 65535;
    }
    if((matchTimeRemaining < 31 && matchTimeRemaining > 30.8) || ( matchTimeRemaining < 30.7 && matchTimeRemaining > 30.5)) {
        left_rumble = 65535;
        right_rumble = 65535;
    }
    if((matchTimeRemaining < 11 && matchTimeRemaining > 10.8)) {
        left_rumble = 65535;
        right_rumble = 65535;
    }
    rumble_type_converterPublish(left_rumble, right_rumble);

}
//TODO: Overrides
//Only allow going to various buttons based on cube state
//Actually organize code so it can be debuged
// 
void evaluate_commands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState) {
	/*std_msgs::Header first_header;
   	first_header.stamp = JoystickState -> header.stamp;
	first_header.seq = 0;
	joystick_test_vel.publish(first_header);*/

	elevator_controller::ElevatorControlS srv_elevator;
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
    elevator_controller::ElevatorControl elevator_msg;
    time_secs = ros::Time::now().toSec();

/*-----------------Up Double Press End Deploy------------------------------*/	

    if(JoystickState->directionUpPress == true && matchTimeRemaining < 30 ) {
		if(time_secs - directionUpLast < 1.0) {
			elevator_controller::Blank msg; //TODO
			msg.request.nothing_here = true;
			end_game_deploy.call(msg);
			ROS_WARN("SELF DESTURCT");
			achieved_pos = other;
		}
		directionUpLast = time_secs;
    }
/*-----------------Down Press Climb to Correct height----------------------*/	
        if(JoystickState->directionDownPress == true) {
            srv_elevator.request.y = climb;
            srv_elevator.request.x = elevator_pos_x;
            srv_elevator.request.up_or_down = elevator_up_or_down;
    	    srv_elevator.request.override_pos_limits = disable_arm_limits;
            elevator_service.call(srv_elevator);
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
                    srv_elevator.request.x = mid_scale_config_x;
                    srv_elevator.request.y = mid_scale_config_y;
                    srv_elevator.request.up_or_down = mid_scale_config_up_or_down;
    	    	    srv_elevator.request.override_pos_limits = disable_arm_limits;
                achieved_pos = mid_scale;
                    if(elevator_service.call(srv_elevator)) {
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
                    if(clamp_service.call(srvClamp)) {
                        ROS_WARN("Clamped");
                        clamped = true;
                    }
                    else {
                        ROS_ERROR("Failed to clamp");
                    }
                }
                else {
                    srvClamp.request.data = false;
                    if(clamp_service.call(srvClamp)) {
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
                if(time_secs - ALast > delay_after_single_tap && time_secs - ALast < max_delay_after_single_tap) {  //wait .3 before assuming single press
		 /*-If in Exchange - Place and Run Intake Out-*/	 
                        if(lastToggle=="YDouble") {

                            ready_to_spin_out_check = true; //Kick off ready to drop
                        }
		 /*-Else - Place and Then Move Arm Back-*/
                        else {
                            srvClamp.request.data=false;
			    clamp_service.call(srvClamp);
			     placed_delay_check = true; //Kick off placing
                             place_start = ros::Time::now().toSec();  
			  }
                        }
                        ALast = 0; //Remove flag
		 	/*Double Press - Just Go Out*/	 
                if(JoystickState->buttonAPress==true) {
                        if(time_secs - ALast < double_tap_zone) {
                            ADoubleStart = time_secs;
			    run_out = true;
                            srvIntake.request.power = -1;
                            srvIntake.request.spring_state = 2; //soft_in
                            srvIntake.request.up = false;
                            intake_service.call(srvIntake);
                            ALast = 0;
                        }
                        else {
                            ALast = time_secs;
                        }
                }
            }
	/*------------------No Cube - Single Press Intake-------------------*/	

	 else if(JoystickState->buttonAPress==true)
		{
                        goal.IntakeCube = false;   
                        goal.GoToHeight = false;
                        goal.MoveArmAway = false;
                        goal.IntakeCubeNoLift = true;
                        ac->sendGoal(goal);

			//TODO: eventually, disable above and enable below:
			//Further more, below and the associated stuff should be
			//Ported to an action lib (a new one)
			//srvIntake.request.power = .8;
                        //srvIntake.request.spring_state = 2; //soft_in
                        //srvIntake.request.up = false;
                        //intake_service.call(srvIntake);
			//go_to_intake_config = achieved_pos != intake;
			//manage_intaking = true;

            }
	
	/*-If in Exchange - Place and Run Intake Out-*/	 

	static bool finish_spin_out_check = false;
	static double time_start_spin = 0;
	if(ready_to_spin_out_check)
	{
	    if(achieved_pos = intake)
	    {
		srvClamp.request.data = false;
		clamp_service.call(srvClamp);

		srvIntake.request.power = -1;
		srvIntake.request.spring_state = 2; //soft_in
		srvIntake.request.up = false;
		intake_service.call(srvIntake);
		finish_spin_out_check = true;		
                ready_to_spin_out_check = false;  
		time_start_spin = ros::Time::now().toSec();
	    }
	}
	
	/*-If just spun out, place and Run Intake Out-*/	 
	if(finish_spin_out_check && (time_secs - time_start_spin) > 2.0)
	{
		srvIntake.request.power = 0;
                srvIntake.request.spring_state = 2; //soft_in
                srvIntake.request.up = false;
                intake_service.call(srvIntake);
                finish_spin_out_check = false;
		
                srv_elevator.request.x = default_x;
                srv_elevator.request.y = default_y;
                srv_elevator.request.up_or_down = default_up_or_down;
    	        srv_elevator.request.override_pos_limits = disable_arm_limits;
                elevator_service.call(srv_elevator);
	    	achieved_pos = default_c;
	}
	static bool return_to_intake_from_low = false;
	static bool return_to_intake_from_high = false;
	static double return_to_intake_start = 0;
	/*If some time has passed since place, start arm move back*/	
	if(placed_delay_check && (time_secs - place_start) > .5)
	{
		srv_elevator.request.x = elevator_pos_x  + move_out_pos_x; 
		srv_elevator.request.y = elevator_pos_y  + move_out_pos_y; 
                srv_elevator.request.up_or_down = move_out_up_or_down;
                srv_elevator.request.override_pos_limits = disable_arm_limits;
                elevator_service.call(srv_elevator);
		placed_delay_check = false;
                return_to_intake_from_low = achieved_pos == switch_c;
		return_to_intake_from_high = !return_to_intake_from_low;
		return_to_intake_start = time_secs;
		achieved_pos = other;
	}
	if(return_to_intake_from_high && time_secs - return_to_intake_start > 2)
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
	if(return_to_intake_from_low && time_secs - return_to_intake_start > 2)
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
		clamp_service.call(srvClamp);	
		srvIntake.request.power = 0.0;
		srvIntake.request.spring_state = 1; //hard_out
		srvIntake.request.up = false;
		go_to_default = true;
		clamp_grab_time = time_secs;		

	}
	/*Go to the normal resting point for a cube*/	
	if(go_to_default && time_secs - clamp_grab_time > .75)
	{			
                srv_elevator.request.x = default_x;
                srv_elevator.request.y = default_y;
                srv_elevator.request.up_or_down = default_up_or_down;
    	        srv_elevator.request.override_pos_limits = disable_arm_limits;
	}	
	/*-below two ifs are exactly like above two ifs, except without clamp/arm/lift*/
        if(run_out && (time_secs > ADoubleStart + 2)) {
            srvIntake.request.power = 0;
            srvIntake.request.spring_state = 2; //soft_in
            srvIntake.request.up = false;
            intake_service.call(srvIntake);
            run_out = false;
        }
        if(hasCube && !run_out) {
            srvIntake.request.power = .1;
            srvIntake.request.spring_state = 3; //hard_in
            srvIntake.request.up = false;
            intake_service.call(srvIntake);
        }

/*------------------------------------Y------------------------------------*/

    if(hasCube) //No need to go to switch/exchange without cube
    {
	
	/*-----------------Single Press - Switch ---------------------*/	
        
	if(time_secs - YLast > delay_after_single_tap && time_secs - YLast < max_delay_after_single_tap) {
            currentToggle = "Yone";
            if(lastToggle==" ") {
                setHeight();
            }
            if(currentToggle == lastToggle) {
                unToggle();
            }
            else {
                srv_elevator.request.x = switch_config_x;
                srv_elevator.request.y = switch_config_y;
                srv_elevator.request.up_or_down = switch_config_up_or_down;
    	        srv_elevator.request.override_pos_limits = disable_arm_limits;
                if(elevator_service.call(srv_elevator)) {
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
            if(time_secs - YLast < .2) {
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
                YLast = time_secs;
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
                srv_elevator.request.x = default_x;
                srv_elevator.request.y = default_y;
                srv_elevator.request.up_or_down = default_up_or_down;
    	        srv_elevator.request.override_pos_limits = disable_arm_limits;
		achieved_pos = default_c;
		if(elevator_service.call(srv_elevator)) {
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
            intake_service.call(srvIntake);
        }
/*-----------------------------------------B------------------------------------------*/
    if(hasCube) //No need to go to switch/exchange without cube
    {
	
	/*--------------------Single Press - Low Scale ---------------------*/	
        if(time_secs - BLast > .21 && time_secs - BLast < .45) {
            currentToggle = "Bone";
            if(lastToggle==" ") {
                setHeight();
            }
            if(currentToggle == lastToggle) {
                unToggle();
            }
            else {
                srv_elevator.request.x = low_scale_config_x;
                srv_elevator.request.y = low_scale_config_y;
                srv_elevator.request.up_or_down = low_scale_config_up_or_down;
    	        srv_elevator.request.override_pos_limits = disable_arm_limits;
		achieved_pos = low_scale;
		
                if(elevator_service.call(srv_elevator)) {
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
            if(time_secs - BLast < .2) {
                currentToggle = "doubleB";
                if(lastToggle==" ") {
                    setHeight();
                }
                if(currentToggle == lastToggle) {
                    unToggle();
                }
                else {
                    srv_elevator.request.x = high_scale_config_x;
                    srv_elevator.request.y = high_scale_config_y;
                    srv_elevator.request.up_or_down = high_scale_config_up_or_down;
    	            srv_elevator.request.override_pos_limits = disable_arm_limits;
		    achieved_pos = high_scale;
                    if(elevator_service.call(srv_elevator)) {
                        ROS_WARN("Toggled to high level scale");
                    }
                    else{
                        ROS_ERROR("Failed to toggle to high level scale");
                    }
                }
                BLast = 0;

            }
            else {
                BLast = time_secs;
            }
        }
   }
/*------------------------Back Button - Hold Spin Out------------------------------------*/
        if(JoystickState->buttonBackButton==true) {
            srvIntake.request.power = -.8;
            srvIntake.request.up = false;
            srvIntake.request.spring_state = 2; //soft_in
            intake_service.call(srvIntake);
        }
        if(JoystickState->buttonBackRelease==true) {
            srvIntake.request.power = 0;
            srvIntake.request.up = false;
            intake_service.call(srvIntake);
        }

   //}


///////////////////// Drivetrain and Elevator Control \\\\\\\\\\\\\\\\\\\


    //Publish drivetrain messages and elevator/pivot
    geometry_msgs::Twist vel;
    //talon_controllers::CloseLoopControllerMsg arm;
    double left_stick_x = (pow(dead_zone_check(JoystickState->left_stick_x), joystick_scale))*max_speed;
    double left_stick_y = (-pow(dead_zone_check(JoystickState->left_stick_y), joystick_scale))*max_speed;

    double right_stick_x = (pow(dead_zone_check(JoystickState->right_stick_x), joystick_scale));
    double right_stick_y = (-pow(dead_zone_check(JoystickState->right_stick_y), joystick_scale));

    Eigen::Vector2d joyVector;
    
    vel.angular.z = (JoystickState->leftTrigger - JoystickState->rightTrigger)*max_rot;
    vel.angular.x = 0;
    vel.angular.y = 0;
    
    if(JoystickState-> bumperLeftButton == true) {
        left_stick_x *= slow_mode;
        left_stick_y *= slow_mode;

        right_stick_x *= slow_mode;
        right_stick_y *= slow_mode;
        
	vel.angular.z *= slow_mode;
    }
    if((fabs(left_stick_x) == 0.0 && fabs(left_stick_y) == 0.0 && fabs(vel.angular.z) == 0.0) && !sendRobotZero) {
        talon_swerve_drive_controller::Blank blank;
        blank.request.nothing = true;
        brake_srv.call(blank);
        sendRobotZero = true;
    }
    else if(fabs(left_stick_x) != 0.0 || fabs(left_stick_y) != 0.0 || fabs(vel.angular.z) != 0.0)
    {


    joyVector[0] = left_stick_x; //intentionally flipped
    joyVector[1] = -left_stick_y;
    Eigen::Rotation2Dd r(-navX_angle_ - M_PI/2);
    Eigen::Vector2d rotatedJoyVector = r.toRotationMatrix() * joyVector;
	vel.linear.x = rotatedJoyVector[1];
	vel.linear.y = rotatedJoyVector[0];
        joystick_robot_vel.publish(vel);
	/*std_msgs::Header test_header;
	test_header.stamp = JoystickState -> header.stamp;
	test_header.seq = 1;
	joystick_test_vel.publish(test_header);*/
	sendRobotZero = false;
    }
    if(right_stick_x != 0 && right_stick_y != 0) {
        elevator_msg.x = elevator_pos_x + (time_secs-last_time_secs)*right_stick_x; //Access current elevator position to fix code allowing out of bounds travel
        elevator_msg.y = elevator_pos_y + (time_secs-last_time_secs)*right_stick_y; //Access current elevator position to fix code allowing out of bounds travel
	elevator_msg.up_or_down = elevator_up_or_down;
	elevator_msg.override_pos_limits = disable_arm_limits; 
        joystick_elevator_pos.publish(elevator_msg);
    }

    last_time_secs = time_secs;
}

void odom_callback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
   elevator_pos_x =  msg->x;
   elevator_pos_y =  msg->y;
   elevator_up_or_down =msg->up_or_down;
}
/*
void evaluateState(const teleop_joystick_control::RobotState::ConstPtr &RobotState) {
    if(RobotState->ifCube==true) {
        ifCube = true;
        rumble_type_converterPublish(0, 32767);
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
    uint16_t left_rumble=0, right_rumble=0;
    double matchTimeRemaining = MatchData->matchTimeRemaining;
	// TODO : make these a set of else if blocks?
    if(matchTimeRemaining < 61 && matchTimeRemaining > 59) {
        left_rumble = 65535;
    }
    else if(matchTimeRemaining < 31 && matchTimeRemaining > 29) {
        right_rumble = 65535;
    }
    else if(matchTimeRemaining <17 && matchTimeRemaining > 14) {
        allback(const sensor_msgs::Imu &navXState)

    }
    rumble_type_converterPublish(left_rumble, right_rumble);
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

    joystick_robot_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joystick_test_vel = n.advertise<std_msgs::Header>("test_header", 3);
    joystick_elevator_pos = n.advertise<elevator_controller::ElevatorControl>("/frcrobot/elevator_controller/cmd_pos", 1);
    joystick_rumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);

    end_game_deploy = n.serviceClient<elevator_controller::Blank>("/frcrobot/elevator_controller/end_game_deploy", 1);
    elevator_service = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
    clamp_service = n.serviceClient<elevator_controller::bool_srv>("/frcrobot/elevator_controller/clamp");
    intake_service = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
    brake_srv = n.serviceClient<talon_swerve_drive_controller::Blank>("/frcrobot/talon_swerve_drive_controller/brake");

    //message_filters::Subscriber<ros_control_boilerplate::JoystickState> joystickSub(n, "scaled_joystick_vals", 5);
    //message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> matchDataSub(n, "match_data", 5);

    joystick_sub = n.subscribe("joystick_states", 1, &evaluate_commands);
    match_data = n.subscribe("match_data", 1, &match_data_callback);

    navX_heading_ = n.subscribe("/frcrobot/navx_mxp", 1, &navX_callback);
    elevator_odom = n.subscribe("/frcrobot/elevator_controller/odom", 1, &odom_callback);
    cube_state_   = n.subscribe("/frcrobot/elevator_controller/cube_state", 1, &cube_callback);
    disable_arm_limits_sub = n.subscribe("frcrobot/override_arm_limits", 1, &overrideCallback);

	disable_arm_limits = false;
	navX_angle_ = M_PI/2; 

    ROS_WARN("joy_init");

    //typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::JoystickState, ros_control_boilerplate::MatchSpecificData> JoystickSync;
    //message_filters::Synchronizer<JoystickSync> sync(JoystickSync(5), joystickSub, matchDataSub);
    //sync.registerCallback(boost::bind(&evaluate_commands, _1, _2));

    //ac->waitForServer(); //Will wait for infinite time for server to start

    ros::spin();
    return 0;
}












void rumble_type_converterPublish(uint16_t left_rumble, uint16_t right_rumble) {
    unsigned int rumble = ((left_rumble & 0xFFFF) << 16) | (right_rumble & 0xFFFF);
    double rumble_val;
    rumble_val = *((double*)(&rumble));
    std_msgs::Float64 rumble_msg;
    rumble_msg.data = rumble_val;
    joystick_rumble.publish(rumble_msg);
}
/*
void rumble_type_converterPublish(uint16_t left_rumble, uint16_t right_rumble) {
    unsigned int rumble = ((left_rumble & 0xFFFF) << 16) | (right_rumble & 0xFFFF);
    double rumble_val;
    rumble_val = *((double*)(&rumble));
    std_msgs::Float64 rumble_msg;
    rumble_msg.data = rumble_val;
    joystick_rumble.publish(rumble_msg);
}
*/
void navX_callback(const sensor_msgs::Imu &navXState)
{
	tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);
	navX_angle_ = yaw;
}
void cube_callback(const std_msgs::Bool &cube)
{
	hasCube = cube.data;
}
void overrideCallback(const std_msgs::Bool &override_lim)
{
	disable_arm_limits = override_lim.data;
}
