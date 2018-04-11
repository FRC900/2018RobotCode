/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Original Author: Dave Coleman
Desc:   Example ros_control hardware interface blank template for the FRCRobot
For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <ros_control_boilerplate/frcrobot_sim_interface.h>
#include <ros_control_boilerplate/nextVelocity.h>
#include <ros_control_boilerplate/nextVelocity.h>
#include <ros_control_boilerplate/JoystickState.h>
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#define KEYCODE_a 0x61
#define KEYCODE_b 0x62
#define KEYCODE_c 0x63
#define KEYCODE_d 0x64
#define KEYCODE_e 0x65
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6a
#define KEYCODE_k 0x6b
#define KEYCODE_l 0x6c
#define KEYCODE_m 0x6d
#define KEYCODE_n 0x6e
#define KEYCODE_o 0x6f
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_s 0x73
#define KEYCODE_t 0x7
#define KEYCODE_u 0x75
#define KEYCODE_v 0x76
#define KEYCODE_w 0x77
#define KEYCODE_x 0x78
#define KEYCODE_y 0x79
#define KEYCODE_z 0x7a
#define KEYCODE_A 0x41
#define KEYCODE_B 0x42
#define KEYCODE_C 0x43
#define KEYCODE_D 0x44
#define KEYCODE_MINUS 0x2D
#define KEYCODE_EQUALS 0x3D
#define KEYCODE_ONE 0x31
#define KEYCODE_TWO 0x32
#define KEYCODE_THREE 0x33
#define KEYCODE_FOUR 0x34
#define KEYCODE_FIVE 0x35
#define KEYCODE_SIX 0x36
#define KEYCODE_SEVEN 0x37
#define KEYCODE_EIGHT 0x38
#define KEYCODE_LEFT_BRACKET 0x5B
#define KEYCODE_ESCAPE  0x1B
#define KEYCODE_CARROT 0x5E
#define KEYCODE_SPACE 0x20


namespace frcrobot_control
{

FRCRobotSimInterface::FRCRobotSimInterface(ros::NodeHandle &nh,
		urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
}
FRCRobotSimInterface::~FRCRobotSimInterface()
{
    sim_joy_thread_.join();
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
    {
        custom_profile_threads_[i].join();
    }
}

int kfd = 0;
struct termios cooked, raw;

void quit(int /*sig*/)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}

class TeleopJointsKeyboard
{
	public:

		TeleopJointsKeyboard()
			: has_recieved_joints_(false)
		{
			std::cout << "init " << std::endl;
			// TODO: make this robot agonistic
			joints_pub_ = nh_.advertise<ros_control_boilerplate::JoystickState>("/frcrobot/joystick_states", 1);
		}

		~TeleopJointsKeyboard()
		{ }

		void keyboardLoop()
		{
			char c;
			// get the console in raw mode
			tcgetattr(kfd, &cooked);
			memcpy(&raw, &cooked, sizeof(struct termios));
			raw.c_lflag &= ~ (ICANON | ECHO);
			// Setting a new line, then end of file
			raw.c_cc[VEOL] = 1;
			raw.c_cc[VEOF] = 2;
			tcsetattr(kfd, TCSANOW, &raw);

			for (;;)
			{
				// get the next event from the keyboard
				if (read(kfd, &c, 1) < 0)
				{
					perror("read():");
					exit(-1);
				}

                cmd_.rightStickY = 0;
                cmd_.rightStickX = 0;

                cmd_.leftStickY = 0;
                cmd_.leftStickX = 0;

                cmd_.leftTrigger = 0;
                cmd_.rightTrigger = 0;
                cmd_.buttonXButton = false;
                cmd_.buttonXPress = false;
                cmd_.buttonXRelease = false;
                cmd_.buttonYButton = false;
                cmd_.buttonYPress = false;
                cmd_.buttonYRelease = false;

                cmd_.bumperLeftButton = false;
                cmd_.bumperLeftPress = false;
                cmd_.bumperLeftRelease = false;

                cmd_.bumperRightButton = false;
                cmd_.bumperRightPress = false;
                cmd_.bumperRightRelease = false;

                cmd_.stickLeftButton = false;
                cmd_.stickLeftPress = false;
                cmd_.stickLeftRelease = false;

                cmd_.stickRightButton = false;
                cmd_.stickRightPress = false;
                cmd_.stickRightRelease = false;

                cmd_.buttonAButton = false;
                cmd_.buttonAPress = false;
                cmd_.buttonARelease = false;
                cmd_.buttonBButton = false;
                cmd_.buttonBPress = false;
                cmd_.buttonBRelease = false;
                cmd_.buttonBackButton = false;
                cmd_.buttonBackPress = false;
                cmd_.buttonBackRelease = false;

                cmd_.buttonStartButton = false;
                cmd_.buttonStartPress = false;
                cmd_.buttonStartRelease = false;

                cmd_.directionUpButton = false;
                cmd_.directionUpPress = false;
                cmd_.directionUpRelease = false;

                cmd_.directionDownButton = false;
                cmd_.directionDownPress = false;
                cmd_.directionDownRelease = false;

                cmd_.directionRightButton = false;
                cmd_.directionRightPress = false;
                cmd_.directionRightRelease = false;

                cmd_.directionLeftButton = false;
                cmd_.directionLeftPress = false;
                cmd_.directionLeftRelease = false;

				bool dirty = false;
				switch (c)
				{
                case KEYCODE_i:
                    cmd_.rightStickY += .5;
                    dirty = true;
                    break;
                case KEYCODE_k:
                    cmd_.rightStickY -= .5;
                    dirty = true;
                    break;
                case KEYCODE_j:
                    cmd_.rightStickX += .5;
                    dirty = true;
                    break;
                case KEYCODE_l:
                    cmd_.rightStickX -= .5;
                    dirty = true;
                    break;

                case KEYCODE_d:
                    cmd_.leftStickY += .5;
                    dirty = true;
                    break;
                case KEYCODE_a:
                    cmd_.leftStickY -= .5;
                    dirty = true;
                    break;
                case KEYCODE_w:
                    cmd_.leftStickX += .5;
                    dirty = true;
                    break;
                case KEYCODE_s:
                    cmd_.leftStickX -= .5;
                    dirty = true;
                    break;
				case KEYCODE_ONE:
                    if(cmd_last_.buttonAButton) {
                        cmd_.buttonAPress = false; // radians
                    }
                    else {
                        cmd_.buttonAPress = true; // radians
                    }
					cmd_.buttonAButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_TWO:
                    if(cmd_last_.buttonBButton) {
                        cmd_.buttonBPress = false; // radians
                    }
                    else {
                        cmd_.buttonBPress = true; // radians
                    }
					cmd_.buttonBButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_THREE:
                    if(cmd_last_.buttonXButton) {
                        cmd_.buttonXPress = false; // radians
                    }
                    else {
                        cmd_.buttonXPress = true; // radians
                    }
					cmd_.buttonXButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_FOUR:
                    if(cmd_last_.buttonYButton) {
                        cmd_.buttonYPress = false; // radians
                    }
                    else {
                        cmd_.buttonYPress = true; // radians
                    }
					cmd_.buttonYButton = true; // radians
                    dirty = true;
					break;
				/*case KEYCODE_q:
					cmd_.leftTrigger = .5; // radians
                    dirty = true;
					break;
				*/
				case KEYCODE_e:
					cmd_.rightTrigger = .5; // radians
                    dirty = true;
					break;
				case KEYCODE_SEVEN:
                    if(cmd_last_.stickLeftButton) {
                        cmd_.stickLeftPress = false; // radians
                    }
                    else {
                        cmd_.stickLeftPress = true; // radians
                    }
					cmd_.stickLeftButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_EIGHT:
                    if(cmd_last_.stickRightButton) {
                        cmd_.stickRightPress = false; // radians
                    }
                    else {
                        cmd_.stickRightPress = true; // radians
                    }
					cmd_.stickRightButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_FIVE:
                    if(cmd_last_.buttonBackButton) {
                        cmd_.buttonBackPress = false; // radians
                    }
                    else {
                        cmd_.buttonBackPress = true; // radians
                    }
					cmd_.buttonBackButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_SIX:
                    if(cmd_last_.buttonStartButton) {
                        cmd_.buttonStartPress = false; // radians
                    }
                    else {
                        cmd_.buttonStartPress = true; // radians
                    }
					cmd_.buttonStartButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_MINUS:
                    if(cmd_last_.bumperLeftButton) {
                        cmd_.bumperLeftPress = false; // radians
                    }
                    else {
                        cmd_.bumperLeftPress = true; // radians
                    }
					cmd_.bumperLeftButton = true; // radians
                    dirty = true;
					break;
				case KEYCODE_EQUALS:
                    if(cmd_last_.bumperRightButton) {
                        cmd_.bumperRightPress = false; // radians
                    }
                    else {
                        cmd_.bumperRightPress = true; // radians
                    }
					cmd_.bumperRightButton = true; // radians
                    dirty = true;
					break;

				case KEYCODE_LEFT_BRACKET:
                    if (read(kfd, &c, 1) < 0)
                    {
                        perror("read():");
                        exit(-1);
                    }
                    switch (c)
                    {
                        case KEYCODE_B:
                            if(cmd_last_.directionDownButton) {
                                cmd_.directionDownPress = false; // radians
                            }
                            else {
                                cmd_.directionDownPress = true; // radians
                            }
                            cmd_.directionDownButton = true;
                            dirty = true;
                            break;
                        case KEYCODE_A:
                            if(cmd_last_.directionUpButton) {
                                cmd_.directionUpPress = false; // radians
                            }
                            else {
                                cmd_.directionUpPress = true; // radians
                            }
                            cmd_.directionUpButton = true;
                            dirty = true;
                            break;
                        case KEYCODE_D:
                            if(cmd_last_.directionLeftButton) {
                                cmd_.directionLeftPress = false; // radians
                            }
                            else {
                                cmd_.directionLeftPress = true; // radians
                            }
                            cmd_.directionLeftButton = true;
                            dirty = true;
                            break;
                        case KEYCODE_C:
                            if(cmd_last_.directionRightButton) {
                                cmd_.directionRightPress = false; // radians
                            }
                            else {
                                cmd_.directionRightPress = true; // radians
                            }
                            cmd_.directionRightButton = true;
                            dirty = true;
                            break;
                    }
					break;

				case  KEYCODE_ESCAPE:
					//std::cout << std::endl;
					//std::cout << "Exiting " << std::endl;
					//quit(0);
					break;
                case KEYCODE_CARROT:
                    ROS_WARN("I'ts a carrot");
                    dirty = false;
                    break;
                case KEYCODE_SPACE:
                    dirty = true;
                } 
                if(cmd_last_.buttonAButton && !cmd_.buttonAButton) {
                    cmd_.buttonARelease = true;
                }
                if(cmd_last_.buttonBButton && !cmd_.buttonBButton) {
                    cmd_.buttonBRelease = true;
                }
                if(cmd_last_.buttonXButton && !cmd_.buttonXButton) {
                    cmd_.buttonXRelease = true;
                }
                if(cmd_last_.buttonYButton && !cmd_.buttonYButton) {
                    cmd_.buttonYRelease = true;
                }
                if(cmd_last_.buttonStartButton && !cmd_.buttonStartButton) {
                    cmd_.buttonARelease = true;
                }
                if(cmd_last_.buttonBackButton && !cmd_.buttonBackButton) {
                    cmd_.buttonARelease = true;
                }
                if(cmd_last_.stickLeftButton && !cmd_.stickLeftButton) {
                    cmd_.stickLeftRelease = true;
                }
                if(cmd_last_.stickRightButton && !cmd_.stickRightButton) {
                    cmd_.stickRightRelease = true;
                }
                if(cmd_last_.bumperLeftButton && !cmd_.bumperLeftButton) {
                    cmd_.bumperLeftRelease = true;
                }
                if(cmd_last_.bumperRightButton && !cmd_.bumperRightButton) {
                    cmd_.bumperRightRelease = true;
                }
                if(cmd_last_.directionRightButton && !cmd_.directionRightButton) {
                    cmd_.directionRightRelease = true;
                }
                if(cmd_last_.directionLeftButton && !cmd_.directionLeftButton) {
                    cmd_.directionLeftRelease = true;
                }
                if(cmd_last_.directionUpButton && !cmd_.directionUpButton) {
                    cmd_.directionUpRelease = true;
                }
                if(cmd_last_.directionDownButton && !cmd_.directionDownButton) {
                    cmd_.directionDownRelease = true;
                }
				// Publish command
				if (dirty)
				{
					// Important safety feature
					//if (!has_recieved_joints_)
					//{
					//	ROS_ERROR_STREAM_NAMED("joint_teleop", "Unable to send joint commands because robot state is invalid");
					//}
					//else
					//{
						joints_pub_.publish(cmd_);
                        cmd_last_ = cmd_;
					//}
				}
		}
    }

	private:

		ros::NodeHandle nh_;
		ros::Publisher joints_pub_;
		ros_control_boilerplate::JoystickState cmd_;
		ros_control_boilerplate::JoystickState cmd_last_;
		bool has_recieved_joints_;

};

void FRCRobotSimInterface::loop_joy(void)
{
	signal(SIGINT, quit);
    TeleopJointsKeyboard teleop;
    teleop.keyboardLoop();
}

void FRCRobotSimInterface::cube_state_callback(const elevator_controller::CubeState &cube) {
    clamp = cube.clamp;
    intake_high = cube.intake_high;
    intake_low = cube.intake_low;
    has_cube = cube.has_cube;
}

void FRCRobotSimInterface::match_data_callback(const ros_control_boilerplate::MatchSpecificData &match_data) {
	match_data_enabled_.store(match_data.isEnabled, std::memory_order_relaxed);
}

void FRCRobotSimInterface::init(void)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	ROS_WARN("Passes");	
	FRCRobotInterface::init();
	ROS_WARN("Passes");	
    ros::NodeHandle nh_;

	sim_joy_thread_ = std::thread(&FRCRobotSimInterface::loop_joy, this);
    cube_state_sub_ = nh_.subscribe("/frcrobot/cube_state_sim", 1, &FRCRobotSimInterface::cube_state_callback, this);
    match_data_sub_ = nh_.subscribe("/frcrobot/match_data", 1, &FRCRobotSimInterface::match_data_callback, this);
	
	ROS_WARN("fails here?1");
	// Loop through the list of joint names
	// specified as params for the hardware_interface.
	// For each of them, create a Talon object. This
	// object is used to send and recieve commands
	// and status to/from the physical Talon motor
	// controller on the robot.  Use this pointer
	// to initialize each Talon with various params
	// set for that motor controller in config files.
	// TODO : assert can_talon_srx_names_.size() == can_talon_srx_can_ids_.size()
	for (size_t i = 0; i < can_talon_srx_names_.size(); i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << can_talon_srx_names_[i] <<
							  " as CAN id " << can_talon_srx_can_ids_[i]);
		
		ROS_WARN_STREAM("fails here? 56789: " << i);
	// Loop through the list of joint names
		
		custom_profile_threads_.push_back(std::thread(&FRCRobotSimInterface::custom_profile_thread, this, i));
		ROS_WARN("post and stuff");
	}
		ROS_WARN_STREAM("fails here? ~");
	// TODO : assert nidec_brushles_names_.size() == nidec_brushles_xxx_channels_.size()
	for (size_t i = 0; i < nidec_brushless_names_.size(); i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << nidec_brushless_names_[i] <<
							  " as PWM channel " << nidec_brushless_pwm_channels_[i] <<
							  " / DIO channel " << nidec_brushless_dio_channels_[i] <<
							  " invert " << nidec_brushless_inverts_[i]);

	for (size_t i = 0; i < num_digital_inputs_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << digital_input_names_[i] <<
							  " as Digitial Input " << digital_input_dio_channels_[i] <<
							  " invert " << digital_input_inverts_[i]);

	for (size_t i = 0; i < num_digital_outputs_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << digital_output_names_[i] <<
							  " as Digitial Output " << digital_output_dio_channels_[i] <<
							  " invert " << digital_output_inverts_[i]);

	for (size_t i = 0; i < num_pwm_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << pwm_names_[i] <<
							  " as PWM " << pwm_pwm_channels_[i] <<
							  " invert " << pwm_inverts_[i]);

	for (size_t i = 0; i < num_solenoids_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << solenoid_names_[i] <<
							  " as Solenoid " << solenoid_ids_[i]);

	for (size_t i = 0; i < num_double_solenoids_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << double_solenoid_names_[i] <<
							  " as Double Solenoid  forward " << double_solenoid_forward_ids_[i] <<
							  " reverse " << double_solenoid_reverse_ids_[i]);

	for(size_t i = 0; i < num_navX_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << navX_names_[i] <<
							  " as navX id" << navX_ids_[i]);
	for (size_t i = 0; i < num_analog_inputs_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << analog_input_names_[i] <<
							  " as Analog Input " << analog_input_analog_channels_[i]);

	for (size_t i = 0; i < num_compressors_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << compressor_names_[i] <<
							  " as Compressor with pcm " << compressor_pcm_ids_[i]);
 
	for(size_t i = 0; i < num_dummy_joints_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading dummy joint " << i << "=" << dummy_joint_names_[i]);

	ROS_INFO_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready.");
}

void FRCRobotSimInterface::read(ros::Duration &/*elapsed_time*/)
{
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
        auto &ts = talon_state_[joint_id];
        if(ts.getCANID() == 51) {
            if(clamp) {
                ts.setForwardLimitSwitch(true);
            }
            else {
                ts.setForwardLimitSwitch(false);
            }
        }
    }
    for (size_t i = 0; i < num_digital_inputs_; i++) 
        {    
            //State should really be a bool - but we're stuck using
            //ROS control code which thinks everything to and from
            //hardware are doubles
            if(digital_input_names_[i] == "intake_line_break_high") {
                digital_input_state_[i] = (intake_high) ? 1 : 0; 
            }
            if(digital_input_names_[i] == "intake_line_break_low") {
                digital_input_state_[i] = (intake_low) ? 1 : 0; 
            }
        }    

    // Simulated state is updated in write, so just
	// display it here for debugging

	//printState();
	static bool printed_robot_code_ready;
	if (!printed_robot_code_ready && (robot_code_ready_ != 0.0))
	{
		ROS_WARN("ROBOT CODE READY!");
		printed_robot_code_ready = true;
	}
    ros::spinOnce();
}

void FRCRobotSimInterface::write(ros::Duration &elapsed_time)
{
#if 0
	ROS_INFO_STREAM_THROTTLE(1,
			std::endl << std::string(__FILE__) << ":" << __LINE__ <<
			std::endl << "Command" << std::endl << printCommandHelper());
#endif
	// Was the robot enabled last time write was run?
	static bool last_robot_enabled = false;

	// Is match data reporting the robot enabled now?
	const bool robot_enabled = match_data_enabled_.load(std::memory_order_relaxed);
	
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];

		if(talon_command_[joint_id].getCustomProfileRun())
		{
			can_talon_srx_run_profile_stop_time_[joint_id] = ros::Time::now().toSec();
			continue; //Don't mess with talons running in custom profile mode		
		}
		// If commanded mode changes, copy it over
		// to current state
		hardware_interface::TalonMode new_mode = tc.getMode();

		// Only set mode to requested one when robot is enabled
		if (robot_enabled)
		{
			if (tc.newMode(new_mode))
			{
				ts.setTalonMode(new_mode);
				ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" mode " << (int)new_mode);
			}
			hardware_interface::DemandType demand1_type_internal;
			double demand1_value;
			if (tc.demand1Changed(demand1_type_internal, demand1_value))
			{
				ts.setDemand1Type(demand1_type_internal);
				ts.setDemand1Value(demand1_value);

				ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" demand1 type / value");
			}
		}
		else if (last_robot_enabled)
		{
			// If this is a switch from enabled to
			// disabled, set talon command to current
			// talon mode and then disable the talon. 
			// This will set up the talon to return
			// to the current mode once the robot is 
			// re-enabled
			// Need to first setMode to disabled because there's
			// a check in setMode to see if requested_mode == current_mode
			// If that check is true setMode does nothing - assumes
			// that the mode won't need to be reset back to the
			// same mode it is already in
			tc.setMode(hardware_interface::TalonMode_Disabled);
			tc.setMode(ts.getTalonMode());
			ts.setTalonMode(hardware_interface::TalonMode_Disabled);
		}

		bool close_loop_mode = false;
		bool motion_profile_mode = false;

		// Use requested next talon mode here to update
		// Talon config.  This way settings will be written
		// even if the robot is disabled.  It will also insure
		// that config relevant to the requested mode is
		// written before switching to that mode.
		if ((new_mode == hardware_interface::TalonMode_Position) ||
		    (new_mode == hardware_interface::TalonMode_Velocity) ||
		    (new_mode == hardware_interface::TalonMode_Current ))
		{
			close_loop_mode = true;
		}
		else if ((new_mode == hardware_interface::TalonMode_MotionProfile) ||
			     (new_mode == hardware_interface::TalonMode_MotionMagic))
		{
			close_loop_mode = true;
			motion_profile_mode = true;
		}

		// Only update PID settings if closed loop
		// mode has been requested
		if (close_loop_mode)
		{
			int slot;
			const bool slot_changed = tc.slotChanged(slot);

			double p;
			double i;
			double d;
			double f;
			int   iz;
			int   allowable_closed_loop_error;
			double max_integral_accumulator;
			double closed_loop_peak_output;
			int    closed_loop_period;
			if (tc.pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, closed_loop_peak_output, closed_loop_period, slot) ||  ros::Time::now().toSec()- can_talon_srx_run_profile_stop_time_[joint_id] < .2)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" PIDF slot " << slot << " config values");
				ts.setPidfP(p, slot);
				ts.setPidfI(i, slot);
				ts.setPidfD(d, slot);
				ts.setPidfF(f, slot);
				ts.setPidfIzone(iz, slot);
				ts.setAllowableClosedLoopError(allowable_closed_loop_error, slot);
				ts.setMaxIntegralAccumulator(max_integral_accumulator, slot);
				ts.setClosedLoopPeakOutput(closed_loop_peak_output, slot);
				ts.setClosedLoopPeriod(closed_loop_period, slot);
			}

			if (slot_changed)
			{
				ts.setSlot(slot);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" PIDF slot to " << slot);
			}
		}
		// Invert / sensor phase matters for all modes
		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" invert / phase");
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
		}

		hardware_interface::NeutralMode neutral_mode;
		if (tc.neutralModeChanged(neutral_mode))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral mode");
			ts.setNeutralMode(neutral_mode);
		}

		if (tc.neutralOutputChanged())
		{
			ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral output");
			ts.setNeutralOutput(true);
		}

		double iaccum;
		if (close_loop_mode && tc.integralAccumulatorChanged(iaccum))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" integral accumulator");
		}

		double closed_loop_ramp;
		double open_loop_ramp;
		double peak_output_forward;
		double peak_output_reverse;
		double nominal_output_forward;
		double nominal_output_reverse;
		double neutral_deadband;
		if (tc.outputShapingChanged(closed_loop_ramp,
									open_loop_ramp,
									peak_output_forward,
									peak_output_reverse,
									nominal_output_forward,
									nominal_output_reverse,
									neutral_deadband))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" output shaping");
			ts.setOpenloopRamp(open_loop_ramp);
			ts.setClosedloopRamp(closed_loop_ramp);
			ts.setPeakOutputForward(peak_output_forward);
			ts.setPeakOutputReverse(peak_output_reverse);
			ts.setNominalOutputForward(nominal_output_forward);
			ts.setNominalOutputReverse(nominal_output_reverse);
			ts.setNeutralDeadband(neutral_deadband);
		}
		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.voltageCompensationChanged(v_c_saturation,
									v_measurement_filter,
									v_c_enable))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" voltage compensation");
			ts.setVoltageCompensationSaturation(v_c_saturation);
			ts.setVoltageMeasurementFilter(v_measurement_filter);
			ts.setVoltageCompensationEnable(v_c_enable);
		}

		hardware_interface::VelocityMeasurementPeriod v_m_period;
		int v_m_window;

		if (tc.velocityMeasurementChanged(v_m_period, v_m_window))
		{
			ts.setVelocityMeasurementPeriod(v_m_period);
			ts.setVelocityMeasurementWindow(v_m_window);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" velocity measurement period / window");
		}
		
		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" selected sensor position");
			ts.setPosition(sensor_position);
		}

		hardware_interface::LimitSwitchSource internal_local_forward_source;
		hardware_interface::LimitSwitchNormal internal_local_forward_normal;
		hardware_interface::LimitSwitchSource internal_local_reverse_source;
		hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
		if (tc.limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
				internal_local_reverse_source, internal_local_reverse_normal))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" limit switches");
			ts.setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
			ts.setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
		}

		double softlimit_forward_threshold;
		bool softlimit_forward_enable;
		double softlimit_reverse_threshold;
		bool softlimit_reverse_enable;
		bool softlimit_override_enable;
		if (tc.SoftLimitChanged(softlimit_forward_threshold,
				softlimit_forward_enable,
				softlimit_reverse_threshold,
				softlimit_reverse_enable,
				softlimit_override_enable))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" soft limits " <<
					std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
					std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
					std::endl << "\toverride_enable=" << softlimit_override_enable);
			ts.setForwardSoftLimitThreshold(softlimit_forward_threshold);
			ts.setForwardSoftLimitEnable(softlimit_forward_enable);
			ts.setReverseSoftLimitThreshold(softlimit_reverse_threshold);
			ts.setReverseSoftLimitEnable(softlimit_reverse_enable);
			ts.setOverrideSoftLimitsEnable(softlimit_override_enable);
		}

		int peak_amps;
		int peak_msec;
		int continuous_amps;
		bool enable;
		if (tc.currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" peak current");
			ts.setPeakCurrentLimit(peak_amps);
			ts.setPeakCurrentDuration(peak_msec);
			ts.setContinuousCurrentLimit(continuous_amps);
			ts.setCurrentLimitEnable(enable);
		}

		if (motion_profile_mode)
		{
			double motion_cruise_velocity;
			double motion_acceleration;
			if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" cruise velocity / acceleration");
				ts.setMotionCruiseVelocity(motion_cruise_velocity);
				ts.setMotionAcceleration(motion_acceleration);
			}

			// Do this before rest of motion profile stuff
			// so it takes effect before starting a buffer?
			int motion_control_frame_period;
			if (tc.motionControlFramePeriodChanged(motion_control_frame_period))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion control frame period");
				ts.setMotionControlFramePeriod(motion_control_frame_period);
			}

			int motion_profile_trajectory_period;
			if (tc.motionProfileTrajectoryPeriodChanged(motion_profile_trajectory_period))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectory period");
				ts.setMotionProfileTrajectoryPeriod(motion_profile_trajectory_period);
			}

			if (tc.clearMotionProfileTrajectoriesChanged())
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");

			if (tc.clearMotionProfileHasUnderrunChanged())
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile underrun changed");

			std::vector<hardware_interface::TrajectoryPoint> trajectory_points;

			if (tc.motionProfileTrajectoriesChanged(trajectory_points))
				ROS_INFO_STREAM("Added " << trajectory_points.size() << " points to joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");
		}

		hardware_interface::TalonMode simulate_mode = ts.getTalonMode();
		if (simulate_mode == hardware_interface::TalonMode_Position)
		{
			// Assume instant velocity
			double position;

			if (tc.commandChanged(position))
				ts.setSetpoint(position);

			ts.setPosition(position);
			ts.setSpeed(0);
		}
		else if (simulate_mode == hardware_interface::TalonMode_Velocity)
		{
			// Assume instant acceleration for now
			double speed;

			if (tc.commandChanged(speed))
				ts.setSetpoint(speed);

			ts.setPosition(ts.getPosition() + speed * elapsed_time.toSec());
			ts.setSpeed(speed);
		}
		else if (simulate_mode == hardware_interface::TalonMode_MotionMagic)
		{
			double setpoint;
		
			if (tc.commandChanged(setpoint))
				ts.setSetpoint(setpoint);

			const double position = ts.getPosition();
			double velocity = ts.getSpeed();
			const double dt = elapsed_time.toSec();

			//change the nextVelocity call to non existent as it does not work and throws an error from a non-existent package
			double next_pos = nextVelocity(position, setpoint, velocity, ts.getMotionCruiseVelocity(), ts.getMotionAcceleration(), dt);
			//ROS_WARN_STREAM("max vel: " <<ts.getMotionCruiseVelocity()<< " max accel: " << ts.getMotionAcceleration());

			//Talons don't allow overshoot, the profiling algorithm above does
			if ((position <= setpoint && setpoint < next_pos) || (position >= setpoint && setpoint > next_pos))
			{
				next_pos = setpoint;
				velocity = 0;
			}
			ts.setPosition(next_pos);
			ts.setSpeed(velocity);
		}
		else if (simulate_mode == hardware_interface::TalonMode_Disabled)
		{
			ts.setSpeed(0); // Don't know how to simulate decel, so just pretend we are stopped
		}

		if (tc.clearStickyFaultsChanged())
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" sticky_faults");
	}
	last_robot_enabled = robot_enabled;

	for (std::size_t joint_id = 0; joint_id < num_nidec_brushlesses_; ++joint_id)
	{
		// Assume instant acceleration for now
		const double vel = brushless_command_[joint_id];
		brushless_vel_[joint_id] = vel;
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		bool converted_command = (digital_output_command_[i] > 0) ^ digital_output_inverts_[i];
		digital_output_state_[i] = converted_command;
	}
	for (size_t i = 0; i < num_pwm_; i++)
	{
		int inverter  = (pwm_inverts_[i]) ? -1 : 1;
		pwm_state_[i] = pwm_command_[i]*inverter;
	}
	for (size_t i = 0; i< num_solenoids_; i++)
	{
		bool setpoint = solenoid_command_[i] > 0;
		solenoid_state_[i] = setpoint;
	}

	for (size_t i = 0; i< num_double_solenoids_; i++)
	{
		// TODO - maybe check for < 0, 0, >0 and map to forward/reverse?
		const double command = double_solenoid_command_[i];
		double setpoint;
		if (command >= 1.)
			setpoint = 1.;
		else if (command <= -1.)
			setpoint = -1.;
		else
			setpoint = 0.;

		double_solenoid_state_[i] = setpoint;
	}
	for (size_t i = 0; i < num_rumble_; i++)
	{
		unsigned int rumbles = *((unsigned int*)(&rumble_command_[i]));
		unsigned int left_rumble  = (rumbles >> 16) & 0xFFFF;
		unsigned int right_rumble = (rumbles      ) & 0xFFFF;
#if 0
		ROS_INFO_STREAM_THROTTLE(1,
				"Joystick at port " << rumble_ports_[i] <<
				" left rumble = " << std::dec << left_rumble << "(" << std::hex << left_rumble <<
				") right rumble = " << std::dec << right_rumble << "(" << std::hex << right_rumble <<  ")" << std::dec);
#endif
	}
	//std::stringstream s;
	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		//s << dummy_joint_command_[i] << " ";
		dummy_joint_effort_[i] = 0;
		//if (dummy_joint_names_[i].substr(2, std::string::npos) == "_angle")
		{
			// position mode
			dummy_joint_velocity_[i] = (dummy_joint_command_[i] - dummy_joint_position_[i]) / elapsed_time.toSec();
			dummy_joint_position_[i] = dummy_joint_command_[i];
		}
		//else if (dummy_joint_names_[i].substr(2, std::string::npos) == "_drive")
		{
			// position mode
			//dummy_joint_position_[i] += dummy_joint_command_[i] * elapsed_time.toSec();
			//dummy_joint_velocity_[i] = dummy_joint_command_[i];
		}
	}
	//ROS_INFO_STREAM_THROTTLE(1, s.str());
}

}  // namespace
