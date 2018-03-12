#include <atomic>
#include <realtime_tools/realtime_buffer.h>
#include "teleop_joystick_control/teleopJoystickCommands.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/Intake.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviors/RobotAction.h"
#include "behaviors/IntakeAction.h"

/*TODO list:
 *
 *
 *Note: arm, end_game_deploy, and intake will become services
 *
 *Press down to climb (lift goes to position)
 *
 */
static double dead_zone = .2, slow_mode = .33, max_speed = 3.6, max_rot = 8.8, joystick_scale = 3;
double dead_zone_check(double val)
{
	if (fabs(val) <= dead_zone)
	{
		return 0;
	}
	return val;
}

std::shared_ptr<actionlib::SimpleActionClient<behaviors::RobotAction>> ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> ac_intake;

// make these params?
const double double_tap_zone = .3;
const double delay_after_single_tap = .31;
const double max_delay_after_single_tap = .45;

static ros::Publisher JoystickRobotVel;
static ros::Publisher JoystickTestVel;
static ros::Publisher JoystickElevatorPos;
static ros::Publisher JoystickRumble;
static ros::ServiceClient EndGameDeploy;

static ros::ServiceClient ElevatorSrv;
static ros::ServiceClient ClampSrv;
static ros::ServiceClient IntakeSrv;
static ros::ServiceClient BrakeSrv;

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

enum pos {high_scale, mid_scale, low_scale, switch_c, exchange, intake_ready_to_drop, intake, intake_low, climb_c, default_c, other};

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

// TODO : initialize values to meaningful defaults
//double elevatorHeight;
// Variables shared between callback threads and main
// teleop callback
static std::atomic<bool> hasCube;

// Elevator odometry. Make this a struct so that
// reads from it get data which is consistent - avoid
// cases where X has been modified by the callback
// but Y hasn't yet.
struct ElevatorPos
{
	ElevatorPos():
		X_(0),
		Y_(0),
		UpOrDown_(false)
	{
	}
	ElevatorPos(double X, double Y, bool UpOrDown) :
		X_(X),
		Y_(Y),
		UpOrDown_(UpOrDown)
	{
	}

	double X_;
	double Y_;
	bool   UpOrDown_;
};
// Use a realtime buffer to store the odom callback data
// The main teleop code isn't technically realtime but we
// want it to be the fast part of the code, so for now
// pretend that is the realtime side of the code
realtime_tools::RealtimeBuffer<ElevatorPos> elevatorPos;

std::atomic<bool> disableArmLimits;
std::atomic<double> navX_angle;
std::atomic<double> matchTimeRemaining;

void unToggle(const pos last_achieved_pos, const ElevatorPos &elevatorPosBefore, pos &achieved_pos, std::string &currentToggle)
{
	currentToggle = " ";
	//TOGGLING BACK TO INTAKE/EXCHANGE CONFIG WON'T FULLY WORK
	elevator_controller::ElevatorControlS srvElevator;

	srvElevator.request.x = elevatorPosBefore.X_;
	srvElevator.request.y = elevatorPosBefore.Y_;
	srvElevator.request.up_or_down = elevatorPosBefore.UpOrDown_;
	srvElevator.request.override_pos_limits = disableArmLimits.load(std::memory_order_relaxed);
	if(!ElevatorSrv.call(srvElevator))
	{
		ROS_ERROR("Untoggle srv call failed");
	}
	ROS_INFO("teleop : called elevatorSrv in unToggle");
	achieved_pos = last_achieved_pos;
}
void setHeight(const pos achieved_pos, pos &last_achieved_pos, ElevatorPos &elevatorPosBefore)
{
	elevatorPosBefore = *(elevatorPos.readFromRT());
	last_achieved_pos = achieved_pos;
}

void match_data_callback(const ros_control_boilerplate::MatchSpecificData::ConstPtr &MatchData)
{
	uint16_t leftRumble = 0, rightRumble = 0;
	//Joystick Rumble
	const double localMatchTimeRemaining = MatchData->matchTimeRemaining;
	matchTimeRemaining.store(localMatchTimeRemaining, std::memory_order_relaxed);

	if ((localMatchTimeRemaining < 91 && localMatchTimeRemaining > 90.8) || ( localMatchTimeRemaining < 90.7 && localMatchTimeRemaining > 90.5) || (localMatchTimeRemaining < 90.4 && localMatchTimeRemaining > 90.2) || ( localMatchTimeRemaining < 90.1 && localMatchTimeRemaining > 89.9) )
	{
		leftRumble = 65535;
		rightRumble = 65535;
	}
	if ((localMatchTimeRemaining < 61 && localMatchTimeRemaining > 60.8) || ( localMatchTimeRemaining < 60.7 && localMatchTimeRemaining > 60.5) || (localMatchTimeRemaining < 60.4 && localMatchTimeRemaining > 60.2))
	{
		leftRumble = 65535;
		rightRumble = 65535;
	}
	if ((localMatchTimeRemaining < 31 && localMatchTimeRemaining > 30.8) || ( localMatchTimeRemaining < 30.7 && localMatchTimeRemaining > 30.5))
	{
		leftRumble = 65535;
		rightRumble = 65535;
	}
	if ((localMatchTimeRemaining < 11 && localMatchTimeRemaining > 10.8))
	{
		leftRumble = 65535;
		rightRumble = 65535;
	}
	rumbleTypeConverterPublish(leftRumble, rightRumble);
}

//TODO: Overrides
//Only allow going to various buttons based on cube state
//Actually organize code so it can be debuged
//
void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState)
{
	/*std_msgs::Header first_header;
	first_header.stamp = JoystickState->header.stamp;
	first_header.seq = 0;
	JoystickTestVel.publish(first_header);*/

	elevator_controller::ElevatorControlS srvElevator;
	std_srvs::SetBool srvClamp;
	elevator_controller::Intake srvIntake;
	static ElevatorPos elevatorPosBefore;

	static bool run_out = false;
    static bool run_in = false;
	behaviors::RobotGoal goal;
    behaviors::IntakeGoal goal_intake;
	goal.IntakeCube = false;
	goal.MoveToIntakeConfig = false;
	goal.x = 0;
	goal.y = 0;
	goal.up_or_down = true;
	goal.override_pos_limits = false;
	elevator_controller::ElevatorControl elevatorMsg;
	const double timeSecs = ros::Time::now().toSec();
	static double lastTimeSecs = 0;
	const bool localHasCube = hasCube.load(std::memory_order_relaxed);
	goal.hasCube = localHasCube;
	const bool localDisableArmLimits = disableArmLimits.load(std::memory_order_relaxed);

	static pos achieved_pos = other; //Not all neccesarily set after achieving, only if needed
	static pos last_achieved_pos = other;

	static std::string currentToggle = " ";
	static std::string lastToggle = " ";

	/*-----------------Up Double Press End Deploy------------------------------*/

	if (JoystickState->directionUpPress && matchTimeRemaining.load(std::memory_order_relaxed) < 30 )
	{
		static double directionUpLast = 0;
		if (timeSecs - directionUpLast < 1.0)
		{
			std_srvs::Empty empty; //TODO
			if (!EndGameDeploy.call(empty))
				ROS_ERROR("EndGameDeploy call in teleop joystick failed");
			ROS_WARN("SELF DESTURCT");
			achieved_pos = other;
		}
		directionUpLast = timeSecs;
	}
	/*-----------------Down Press Climb to Correct height----------------------*/
	if (JoystickState->directionDownPress)
	{
		const ElevatorPos epos = *(elevatorPos.readFromRT());
		srvElevator.request.x = epos.X_; //Consider changing x_pos + up/down to preassigned rather than curr pos
		srvElevator.request.y = climb;
		srvElevator.request.up_or_down = epos.UpOrDown_;
		srvElevator.request.override_pos_limits = localDisableArmLimits;
		srvIntake.request.power = 0;
		srvIntake.request.spring_state = 1; //hard_out
		srvIntake.request.up = false;
		if (!IntakeSrv.call(srvIntake)) //Is it worth trying to clamp or run the intake slowly?
			ROS_ERROR("IntakeSrv call failed in teleop joystick climb config");
		
		if(!ElevatorSrv.call(srvElevator))
		{
			ROS_ERROR("Climb config srv call failed");
		}
		ROS_WARN("Climb config");
		achieved_pos = climb_c;
	}

	//****************************************************\\
	/////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\
	//////////////////////// TOGGLES \\\\\\\\\\\\\\\\\\\\\\\


	lastToggle = currentToggle;




	/*-------------------------------------X------------------------------------*/


	/*---------w/ Cube Single Press Toggle Mid Scale------*/
	if (JoystickState->buttonXPress  )
	{
		static double clamp_time;
		static bool clamped;
		if (localHasCube && (!clamped || timeSecs - clamp_time > 10))
		{
			currentToggle = "X";
			if (lastToggle == " ")
			{
				setHeight(achieved_pos, last_achieved_pos, elevatorPosBefore);
			}
			if (currentToggle == lastToggle)
			{
				unToggle(last_achieved_pos, elevatorPosBefore, achieved_pos, currentToggle);

			}
			else
			{
				srvElevator.request.x = mid_scale_config_x;
				srvElevator.request.y = mid_scale_config_y;
				srvElevator.request.up_or_down = mid_scale_config_up_or_down;
				srvElevator.request.override_pos_limits = localDisableArmLimits;
				achieved_pos = mid_scale;
				if (ElevatorSrv.call(srvElevator))
				{
					ROS_WARN("Toggled to mid level scale height");
				}
				else
				{
					ROS_ERROR("Failed to toggle to mid level scale height");
				}
			}
		}
		/*------------No Cube Single Press Toggle Clamp------*/
		else
		{
			if (clamped == false)
			{
				srvClamp.request.data = true;
				if (ClampSrv.call(srvClamp))
				{
					ROS_WARN("Clamped");
					clamped = true;
					clamp_time = timeSecs;
				}
				else
				{
					ROS_ERROR("Failed to clamp");
				}
			}
			else
			{
				srvClamp.request.data = false;
				if (ClampSrv.call(srvClamp))
				{
					ROS_WARN("UnClamped");
					clamped = false;
				}
				else
				{
					ROS_ERROR("Failed to unclamp");
				}
			}
		}
	}

	/*------------------------------------A/Back button(M2)------------------------------------*/

	/*---------------------w/ Cube------------------------------*/
	static bool ready_to_spin_out_check = false;
	static double place_start = 0;
	static double ADoubleStart = 0;
    static double buttonBackStart = 0;
    static double buttonStartStart = 0;
	static bool placed_delay_check = false;
	static bool manage_intaking;
	static double ALast = 0;
	if (localHasCube)
	{
    /*----------------Single Press------------------*/
        /*-If in Exchange - Place and Run Intake Out-*/
        if (achieved_pos == intake)
        {

            ready_to_spin_out_check = true; //Kick off ready to drop
        }
        /*-Else - Place and Then Move Arm Back-*/
        else
        {
            srvClamp.request.data = false;
            if (!ClampSrv.call(srvClamp))
                ROS_ERROR("ClampSrv call failed in clamp with cube");
            ROS_INFO("teleop : Clamp with cube");
            placed_delay_check = true; //Kick off placing
            place_start = ros::Time::now().toSec();
        }
		ALast = 0; //Remove flag
		/*Back button press(M2) - Just Go Out*/
		if (JoystickState->buttonBackPress == true)
		{
            //Consider changing this functionallity
            buttonBackStart = timeSecs;
            run_out = true;
            srvIntake.request.power = -1;
            srvIntake.request.spring_state = 2; //soft_in
            srvIntake.request.up = false;
            if (!IntakeSrv.call(srvIntake))
                ROS_ERROR("IntakeSrv call failed in intake with cube");
            ROS_INFO("teleop : Intake with cube");
		}
	}
	/*------------------No Cube - Single Press Intake-------------------*/

	else if (JoystickState->buttonAPress == true)
	{
		goal.IntakeCube = true;
		goal.MoveToIntakeConfig = false;
		goal.x = default_x;
		goal.y = default_y;
		goal.up_or_down = default_up_or_down;
		goal.override_pos_limits = false;
		goal.dist_tolerance = .5;
		goal.x_tolerance = .5;
		goal.y_tolerance = .5;
		goal.time_out = 15;

		ac->sendGoal(goal);
		achieved_pos = other;
	}
    /*------------------ Start Button No Cube - Intake No Lift --------------*/

    else if(JoystickState->buttonStartPress) {
        buttonStartStart = timeSecs;
        goal_intake.IntakeCube = true;
        ac_intake->sendGoal(goal_intake);
        ROS_INFO("teleop : Intake No Lift");
    }

	/*-If in Exchange - Place and Run Intake Out-*/

	static bool finish_spin_out_check = false;
	static double time_start_spin = 0;
	if (ready_to_spin_out_check)
	{
		srvClamp.request.data = false;
		if (!ClampSrv.call(srvClamp))
			ROS_ERROR("ClampSrv call failed in ready_to_spin_out_check");
		ROS_INFO("teleop : called ClampSrv in ready_to_spin_out_check");

		srvIntake.request.power = -1;
		srvIntake.request.spring_state = 2; //soft_in
		srvIntake.request.up = false;
		if (!IntakeSrv.call(srvIntake))
			ROS_ERROR("IntakeSrv call failed in ready_to_spin_out_check");
		ROS_INFO("teleop : called IntakeSrv in ready_to_spin_out_check");
		finish_spin_out_check = true;
		ready_to_spin_out_check = false;
		time_start_spin = ros::Time::now().toSec();
	}

	/*-If just spun out, place and Run Intake Out-*/
	if (finish_spin_out_check && (timeSecs - time_start_spin) > 2.0)
	{
		srvIntake.request.power = 0;
		srvIntake.request.spring_state = 2; //soft_in
		srvIntake.request.up = false;
		if (!IntakeSrv.call(srvIntake))
			ROS_ERROR("IntakeSrv call failed in finish_spin_out_check");
		ROS_INFO("teleop : called IntakeSrv in finish_spin_out_check");
		finish_spin_out_check = false;

		goal.IntakeCube = false;
		goal.MoveToIntakeConfig = true;
		goal.time_out = 10;

		ac->sendGoal(goal);

		//TODO: change to going to intake config

		/*
		srvElevator.request.x = default_x;
		srvElevator.request.y = default_y;
		srvElevator.request.up_or_down = default_up_or_down;
		srvElevator.request.override_pos_limits = localDisableArmLimits;

		ElevatorSrv.call(srvElevator);
		*/
		ROS_INFO("teleop : called Go to Intake Goal in finish_spin_out_check");
		achieved_pos = intake;
	}
	static bool return_to_intake_from_low = false;
	static bool return_to_intake_from_high = false;
	static double return_to_intake_start = 0;
	/*If some time has passed since place, start arm move back*/
	if (placed_delay_check && (timeSecs - place_start) > .5)
	{
		const ElevatorPos epos = *(elevatorPos.readFromRT());
		srvElevator.request.x = epos.X_ + move_out_pos_x;
		srvElevator.request.y = epos.Y_ + move_out_pos_y;
		srvElevator.request.up_or_down = move_out_up_or_down;
		srvElevator.request.override_pos_limits = localDisableArmLimits;
		if(!ElevatorSrv.call(srvElevator))
		{
			ROS_ERROR("Failed going up after placing");
		}
		ROS_INFO("teleop : called ElevatorSrv in placed_delay_check");
		placed_delay_check = false;
		return_to_intake_from_low = achieved_pos == switch_c;
		return_to_intake_from_high = !return_to_intake_from_low;
		return_to_intake_start = timeSecs;
		achieved_pos = other;
	}

	if (return_to_intake_from_high && timeSecs - return_to_intake_start > 2)
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
		goal.IntakeCube = false;
		goal.MoveToIntakeConfig = true;
		goal.time_out = 10;
		ac->sendGoal(goal);
		ROS_INFO("teleop : sendGoal() return_to_intake_from_high");
		achieved_pos = intake;
		// TODO : need to call /set goal?
	}

	if (return_to_intake_from_low && timeSecs - return_to_intake_start > 2)
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
		goal.IntakeCube = false;
		goal.MoveToIntakeConfig = true;
		goal.time_out = 10;
		ac->sendGoal(goal);
		ROS_INFO("teleop : sendGoal() return_to_intake_from_low");
		achieved_pos = intake;
		// TODO : need to call /set goal?
	}

	/*When we get the cube, slow the intake and grip hard*/

	if (run_out && (timeSecs > buttonBackStart + 2))
	{
		srvIntake.request.power = 0;
		srvIntake.request.spring_state = 2; //soft_in
		srvIntake.request.up = false;
		if (!IntakeSrv.call(srvIntake))
			ROS_ERROR("IntakeSrv call failed in run_out");
		ROS_INFO("teleop : Intake run out finished");
		run_out = false;
	}

	/*
	if(localHasCube && !run_out) {
	           srvIntake.request.power = .1;
	           srvIntake.request.spring_state = 3; //hard_in
	           srvIntake.request.up = false;
	           IntakeSrv.call(srvIntake);
	       }
	*/

	/*------------------------------------Switch/Exchange------------------------------------*/

	if (localHasCube) //No need to go to switch/exchange without cube
	{

		/*-----------------Left Stick Press - Switch ---------------------*/
        if(JoystickState->stickLeftPress) {
			currentToggle = "StickLeft";
			if (lastToggle == " ")
			{
				setHeight(achieved_pos, last_achieved_pos, elevatorPosBefore);
			}
			if (currentToggle == lastToggle)
			{
				unToggle(last_achieved_pos, elevatorPosBefore, achieved_pos, currentToggle);
			}
			else
			{
				srvElevator.request.x = switch_config_x;
				srvElevator.request.y = switch_config_y;
				srvElevator.request.up_or_down = switch_config_up_or_down;
				srvElevator.request.override_pos_limits =  localDisableArmLimits;
				achieved_pos = switch_c;
				if (ElevatorSrv.call(srvElevator))
				{
					ROS_WARN("Toggled to switch height");
				}
				else
				{
					ROS_ERROR("Failed to toggle to switch height");
				}
			}
		}
		/*-----------------Right Stick Press - Exchange ---------------------*/

        if(JoystickState->stickRightPress) {
            currentToggle = "StickRight";
            if (lastToggle == " ")
            {
                setHeight(achieved_pos, last_achieved_pos, elevatorPosBefore);
            }
            if (currentToggle == lastToggle)
            {
                unToggle(last_achieved_pos, elevatorPosBefore, achieved_pos, currentToggle);
            }
            else
            {
                goal.IntakeCube = false;
                goal.MoveToIntakeConfig = true;
                goal.time_out = 10;

                ac->sendGoal(goal);
                achieved_pos = intake;
            }
        }
	}

	/*----------------------------Right Bumper - Press Untoggle-----------------------------*/
	if (JoystickState->bumperRightPress == true)
	{
		currentToggle = "bumperRight";
		if (lastToggle == " ")
		{
			setHeight(achieved_pos, last_achieved_pos, elevatorPosBefore);
		}
		if (currentToggle == lastToggle)
		{
			unToggle(last_achieved_pos, elevatorPosBefore, achieved_pos, currentToggle);
		}
		else
		{
			if (localHasCube)
			{
				srvElevator.request.x = default_x;
				srvElevator.request.y = default_y;
				srvElevator.request.up_or_down = default_up_or_down;
				srvElevator.request.override_pos_limits = localDisableArmLimits;
				achieved_pos = default_c;
				if (ElevatorSrv.call(srvElevator))
				{
					ROS_WARN("Toggled to default config");
				}
				else
				{
					ROS_ERROR("Failed to toggle to default config");
				}
			}
			else
			{

				goal.IntakeCube = false;
				goal.MoveToIntakeConfig = true;
				goal.time_out = 10;

				ac->sendGoal(goal);
				achieved_pos = intake;

			}
		}
	}
	/*---------------------Start Button - Press Bring Up Intake-------------------*/
	if (JoystickState->directionRightPress == true)
	{
		currentToggle = "right";
		srvIntake.request.power = 0;
		srvIntake.request.up = true;
		srvIntake.request.spring_state = 1; //hard_out
		if (!IntakeSrv.call(srvIntake))
			ROS_ERROR("IntakeSrv call failed in direction right press");
		ROS_INFO("teleop : called IntakeSrv in direction right press");
	}
	/*-----------------------------------------Low/High Scale-----------------------------------------*/
	if (localHasCube) //No need to go to scale without cube
	{
        if(JoystickState->buttonYPress) {
		/*--------------------Y Single Press - Low Scale ---------------------*/
			currentToggle = "Y";
			if (lastToggle == " ")
			{
				setHeight(achieved_pos, last_achieved_pos, elevatorPosBefore);
			}
			if (currentToggle == lastToggle)
			{
				unToggle(last_achieved_pos, elevatorPosBefore, achieved_pos, currentToggle);
			}
			else
			{
				srvElevator.request.x = low_scale_config_x;
				srvElevator.request.y = low_scale_config_y;
				srvElevator.request.up_or_down = low_scale_config_up_or_down;
				srvElevator.request.override_pos_limits = localDisableArmLimits;
				achieved_pos = low_scale;

				if (ElevatorSrv.call(srvElevator))
				{
					ROS_WARN("Toggled to low level scale");
				}
				else
				{
					ROS_ERROR("Failed to toggle to low level scale");
				}
			}
		}
		/*--------------------X Single Press - High Scale ---------------------*/
		if (JoystickState->buttonXPress == true)
		{
            currentToggle = "doubleB";
            if (lastToggle == " ")
            {
                setHeight(achieved_pos, last_achieved_pos, elevatorPosBefore);
            }
            if (currentToggle == lastToggle)
            {
                unToggle(last_achieved_pos, elevatorPosBefore, achieved_pos, currentToggle);
            }
            else
            {
                srvElevator.request.x = high_scale_config_x;
                srvElevator.request.y = high_scale_config_y;
                srvElevator.request.up_or_down = high_scale_config_up_or_down;
                srvElevator.request.override_pos_limits = localDisableArmLimits;
                achieved_pos = high_scale;
                if (ElevatorSrv.call(srvElevator))
                {
                    ROS_WARN("Toggled to high level scale");
                }
                else
                {
                    ROS_ERROR("Failed to toggle to high level scale");
                }
            }

        }
	}
	/*------------------------Back Button - Hold Spin Out------------------------------------*/
	if (JoystickState->buttonBackButton == true)
	{
		srvIntake.request.power = -1;
		srvIntake.request.up = false;
		srvIntake.request.spring_state = 2; //soft_in
		if (!IntakeSrv.call(srvIntake))
			ROS_ERROR("IntakeSrv call failed in back button");
		ROS_INFO("teleop : called IntakeSrv in BackButton press");
	}
	if (JoystickState->buttonBackRelease == true)
	{
		srvIntake.request.power = 0;
		srvIntake.request.up = false;
		if (!IntakeSrv.call(srvIntake))
			ROS_ERROR("IntakeSrv call failed in back button released");
		ROS_INFO("teleop : called IntakeSrv in BackButton release");
	}

	//}

///////////////////// Drivetrain and Elevator Control \\\\\\\\\\\\\\\\\\\

	//talon_controllers::CloseLoopControllerMsg arm;
	double leftStickX = (pow(dead_zone_check(JoystickState->leftStickX), joystick_scale)) * max_speed;
	double leftStickY = (-pow(dead_zone_check(JoystickState->leftStickY), joystick_scale)) * max_speed;

	double rightStickX = pow(dead_zone_check(JoystickState->rightStickX), joystick_scale);
	double rightStickY = -pow(dead_zone_check(JoystickState->rightStickY), joystick_scale);

	double rotation = (JoystickState->leftTrigger - JoystickState->rightTrigger) * max_rot;

	if (JoystickState->bumperLeftButton == true)
	{
		leftStickX *= slow_mode;
		leftStickY *= slow_mode;

		rightStickX *= slow_mode;
		rightStickY *= slow_mode;

		rotation *= slow_mode;
	}

	static bool sendRobotZero = false;
	// No motion? Tell the drive base to stop
	if (fabs(leftStickX) == 0.0 && fabs(leftStickY) == 0.0 && rotation == 0.0)
	{
		if (!sendRobotZero)
		{
			std_srvs::Empty empty;
			if (!BrakeSrv.call(empty))
				ROS_ERROR("BrakeSrv call failed in sendRobotZero");
			//ROS_INFO("teleop : called BrakeSrv to stop");
			/*
			geometry_msgs::Twist vel;
			sendRobotZero += 1;
			
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.linear.z = 0;
	
			vel.angular.x = 0;
			vel.angular.y = 0;
			vel.angular.z = 0;
			
			JoystickRobotVel.publish(vel);
			*/
			ROS_INFO("teleop : called BrakeSrv to stop");
			sendRobotZero = true;
		}
	}
	else // X or Y or rotation != 0 so tell the drive base to move
	{
		sendRobotZero = false;
		//Publish drivetrain messages and elevator/pivot
		Eigen::Vector2d joyVector;
		joyVector[0] = leftStickX; //intentionally flipped
		joyVector[1] = -leftStickY;
		Eigen::Rotation2Dd r(-navX_angle.load(std::memory_order_relaxed) - M_PI / 2);
		Eigen::Vector2d rotatedJoyVector = r.toRotationMatrix() * joyVector;

		geometry_msgs::Twist vel;
		vel.linear.x = rotatedJoyVector[1];
		vel.linear.y = rotatedJoyVector[0];
		vel.linear.z = 0;

		vel.angular.x = 0;
		vel.angular.y = 0;
		vel.angular.z = rotation;

		JoystickRobotVel.publish(vel);
		/*std_msgs::Header test_header;
		  test_header.stamp = JoystickState->header.stamp;
		test_header.seq = 1;
		JoystickTestVel.publish(test_header);*/
		sendRobotZero = false;
		ROS_INFO_STREAM("publishing");
	}

	if (rightStickX != 0 && rightStickY != 0)
	{
		const ElevatorPos epos = *(elevatorPos.readFromRT());
		elevatorMsg.x = epos.X_ + (timeSecs - lastTimeSecs) * rightStickX; //Access current elevator position to fix code allowing out of bounds travel
		elevatorMsg.y = epos.Y_ + (timeSecs - lastTimeSecs) * rightStickY; //Access current elevator position to fix code allowing out of bounds travel
		elevatorMsg.up_or_down = epos.UpOrDown_;
		elevatorMsg.override_pos_limits = localDisableArmLimits;
		JoystickElevatorPos.publish(elevatorMsg);
		ROS_INFO("teleop : Joystive elevator pos");
	}

	lastTimeSecs = timeSecs;
    //ROS_WARN("Header time: %f, Time now: %f, Time difference: %f", JoystickState->header.stamp.toSec(), ros::Time::now().toSec(), ros::Time::now().toSec() - JoystickState->header.stamp.toSec());
}

void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg)
{
	elevatorPos.writeFromNonRT(ElevatorPos(msg->x, msg->y, msg->up_or_down));
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
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;

	ros::NodeHandle n_params(n, "teleop_params");

	if (!n_params.getParam("move_out_pos_x", move_out_pos_x))
		ROS_ERROR("Could not read move_out_pos_x");
	if (!n_params.getParam("move_out_pos_y", move_out_pos_y))
		ROS_ERROR("Could not read move_out_pos_y");
	if (!n_params.getParam("move_out_up_or_down", move_out_up_or_down))
		ROS_ERROR("Could not read move_out_up_or_down");
	if (!n_params.getParam("move_out_down_y", move_out_down_y))
		ROS_ERROR("Could not read move_out_down_y");
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

	ac = std::make_shared<actionlib::SimpleActionClient<behaviors::RobotAction>>("auto_interpreter_server", true);
    
	ac_intake = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("auto_interpreter_server_intake", true);

	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	JoystickTestVel = n.advertise<std_msgs::Header>("test_header", 3);
	JoystickElevatorPos = n.advertise<elevator_controller::ElevatorControl>("/frcrobot/elevator_controller/cmd_pos", 1);
	JoystickRumble = n.advertise<std_msgs::Float64>("rumble_controller/command", 1);

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	EndGameDeploy = n.serviceClient<std_srvs::Empty>("/frcrobot/elevator_controller/end_game_deploy", false, service_connection_header);
	ElevatorSrv = n.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
	ClampSrv = n.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
	IntakeSrv = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot/swerve_drive_controller/brake", false, service_connection_header);

	//message_filters::Subscriber<ros_control_boilerplate::JoystickState> joystickSub(n, "scaled_joystick_vals", 5);
	//message_filters::Subscriber<ros_control_boilerplate::MatchSpecificData> matchDataSub(n, "match_data", 5);

	ros::Subscriber joystick_sub  = n.subscribe("joystick_states", 1, &evaluateCommands);
	ros::Subscriber match_data    = n.subscribe("match_data", 1, &match_data_callback);

	ros::Subscriber navX_heading  = n.subscribe("/frcrobot/navx_mxp", 1, &navXCallback);
	ros::Subscriber elevator_odom = n.subscribe("/frcrobot/elevator_controller/odom", 1, &OdomCallback);
	ros::Subscriber cube_state    = n.subscribe("/frcrobot/elevator_controller/cube_state", 1, &cubeCallback);
	ros::Subscriber diable_arm_limits_sub = n.subscribe("frcrobot/override_arm_limits", 1, &overrideCallback);

	disableArmLimits = false;
	navX_angle = M_PI / 2;
	matchTimeRemaining = std::numeric_limits<double>::max();

	ROS_WARN("joy_init");

	//typedef message_filters::sync_policies::ApproximateTime<ros_control_boilerplate::JoystickState, ros_control_boilerplate::MatchSpecificData> JoystickSync;
	//message_filters::Synchronizer<JoystickSync> sync(JoystickSync(5), joystickSub, matchDataSub);
	//sync.registerCallback(boost::bind(&evaluateCommands, _1, _2));

	//ac->waitForServer(); //Will wait for infinite time for server to start

	ros::spin();
	return 0;
}

void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble)
{
	const unsigned int rumble = ((leftRumble & 0xFFFF) << 16) | (rightRumble & 0xFFFF);
	const double rumble_val = *((double *)&rumble);
	std_msgs::Float64 rumbleMsg;
	rumbleMsg.data = rumble_val;
	JoystickRumble.publish(rumbleMsg);
}

void navXCallback(const sensor_msgs::Imu &navXState)
{
	tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);
	navX_angle.store(yaw, std::memory_order_relaxed);
}
void cubeCallback(const std_msgs::Bool &cube)
{
	hasCube.store(cube.data, std::memory_order_relaxed);
}
void overrideCallback(const std_msgs::Bool &override_lim)
{
	disableArmLimits.store(override_lim.data, std::memory_order_relaxed);
}
