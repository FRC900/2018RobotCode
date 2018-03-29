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

#include <cmath>
#include <iostream>
#include <math.h>
#include <thread>

// ROS message types
#include "ros_control_boilerplate/AutoMode.h"
#include "ros_control_boilerplate/frcrobot_hw_interface.h"
#include "ros_control_boilerplate/JoystickState.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "ros_control_boilerplate/PDPData.h"

#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Matrix3x3.h>

//HAL / wpilib includes
#include "HAL/DriverStation.h"
#include "HAL/HAL.h"
#include "HAL/PDP.h"
#include "HAL/Ports.h"
#include "Joystick.h"
#include <networktables/NetworkTable.h>
#include <SmartDashboard/SmartDashboard.h>

#include <ctre/phoenix/MotorControl/SensorCollection.h>

namespace frcrobot_control
{
const int pidIdx = 0; //0 for primary closed-loop, 1 for cascaded closed-loop
const int timeoutMs = 0; //If nonzero, function will wait for config success and report an error if it times out. If zero, no blocking or checking is performed

FRCRobotHWInterface::FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
}

FRCRobotHWInterface::~FRCRobotHWInterface()
{
	hal_thread_.join();
	motion_profile_thread_.join();

	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		custom_profile_threads_[i].join();
	}
}

// Loop running a basic iterative robot. Used to show robot code ready,
// and then read joystick, match data and network tables values
// during the match.
void FRCRobotHWInterface::hal_keepalive_thread(void)
{
	// This will be written by the last controller to be
	// spawned - waiting here prevents the robot from
	// report robot code ready to the field until
	// all controllers are started
	{
		ros::Rate rate(20);
		while (robot_code_ready_ == 0.0)
			rate.sleep();
	}

	bool joystick_up_ = false;;
	bool joystick_down_ = false;;
	bool joystick_left_ = false;;
	bool joystick_right_ = false;;
	bool joystick_up_last_ = false;;
	bool joystick_down_last_ = false;;
	bool joystick_left_last_ = false;;
	bool joystick_right_last_ = false;;

	robot_.StartCompetition();
	Joystick joystick(0);
	realtime_tools::RealtimePublisher<ros_control_boilerplate::JoystickState> realtime_pub_joystick(nh_, "joystick_states", 1);
	realtime_tools::RealtimePublisher<ros_control_boilerplate::MatchSpecificData> realtime_pub_match_data(nh_, "match_data", 1);
	realtime_tools::RealtimePublisher<ros_control_boilerplate::AutoMode> realtime_pub_nt(nh_, "autonomous_mode", 4);

	// Setup writing to a network table that already exists on the dashboard
	//std::shared_ptr<nt::NetworkTable> pubTable = NetworkTable::GetTable("String 9");
	//std::shared_ptr<nt::NetworkTable> subTable = NetworkTable::GetTable("Custom");
    realtime_pub_nt.msg_.mode.resize(4);
    realtime_pub_nt.msg_.delays.resize(4);
	ros::Time last_nt_publish_time = ros::Time::now();
	//ros::Time last_joystick_publish_time = ros::Time::now();
	ros::Time last_match_data_publish_time = ros::Time::now();

	const double nt_publish_rate = 10;
	//const double joystick_publish_rate = 20;
	const double match_data_publish_rate = 1.1;
	bool game_specific_message_seen = false;

	while (ros::ok())
	{
		robot_.OneIteration();
		const ros::Time time_now_t = ros::Time::now();
		//ROS_INFO("%f", ros::Time::now().toSec());
		// Network tables work!
		//pubTable->PutString("String 9", "WORK");
		//subTable->PutString("Auto Selector", "Select Auto");
		if ((last_nt_publish_time + ros::Duration(1.0 / nt_publish_rate)) < time_now_t)
		{
			// SmartDashboard works!
			frc::SmartDashboard::PutNumber("navX_angle", navX_angle_.load(std::memory_order_relaxed));
			frc::SmartDashboard::PutNumber("Pressure", pressure_.load(std::memory_order_relaxed));
			frc::SmartDashboard::PutBoolean("cube_state", cube_state_.load(std::memory_order_relaxed));

			std::shared_ptr<nt::NetworkTable> driveTable = NetworkTable::GetTable("SmartDashboard");  //Access Smart Dashboard Variables
			if (driveTable && realtime_pub_nt.trylock()) 
			{
				realtime_pub_nt.msg_.mode[0] = (int)driveTable->GetNumber("auto_mode_0", 0);
				realtime_pub_nt.msg_.mode[1] = (int)driveTable->GetNumber("auto_mode_1", 0);
				realtime_pub_nt.msg_.mode[2] = (int)driveTable->GetNumber("auto_mode_2", 0);
				realtime_pub_nt.msg_.mode[3] = (int)driveTable->GetNumber("auto_mode_3", 0);
				realtime_pub_nt.msg_.delays[0] = (int)driveTable->GetNumber("delay_0", 0);
				realtime_pub_nt.msg_.delays[1] = (int)driveTable->GetNumber("delay_1", 0);
				realtime_pub_nt.msg_.delays[2] = (int)driveTable->GetNumber("delay_2", 0);
				realtime_pub_nt.msg_.delays[3] = (int)driveTable->GetNumber("delay_3", 0);
				realtime_pub_nt.msg_.position = (int)driveTable->GetNumber("robot_start_position", 0);
				
				frc::SmartDashboard::PutNumber("auto_mode_0_ret", realtime_pub_nt.msg_.mode[0]);
				frc::SmartDashboard::PutNumber("auto_mode_1_ret", realtime_pub_nt.msg_.mode[1]);
				frc::SmartDashboard::PutNumber("auto_mode_2_ret", realtime_pub_nt.msg_.mode[2]);
				frc::SmartDashboard::PutNumber("auto_mode_3_ret", realtime_pub_nt.msg_.mode[3]);
				frc::SmartDashboard::PutNumber("delay_0_ret", realtime_pub_nt.msg_.delays[0]);
				frc::SmartDashboard::PutNumber("delay_1_ret", realtime_pub_nt.msg_.delays[1]);
				frc::SmartDashboard::PutNumber("delay_2_ret", realtime_pub_nt.msg_.delays[2]);
				frc::SmartDashboard::PutNumber("delay_3_ret", realtime_pub_nt.msg_.delays[3]);
				frc::SmartDashboard::PutNumber("robot_start_position_ret", realtime_pub_nt.msg_.position);

				realtime_pub_nt.msg_.header.stamp = time_now_t;
				realtime_pub_nt.unlockAndPublish();
			}

			if (driveTable)
			{
				disable_compressor_.store((bool)driveTable->GetBoolean("disable_reg", 0), std::memory_order_relaxed);
				frc::SmartDashboard::PutBoolean("disable_reg_ret", disable_compressor_.load(std::memory_order_relaxed));

				override_arm_limits_.store((bool)driveTable->GetBoolean("disable_arm_limits", 0), std::memory_order_relaxed);
				frc::SmartDashboard::PutBoolean("disable_arm_limits_ret", override_arm_limits_.load(std::memory_order_relaxed));

				stop_arm_.store((bool)driveTable->GetBoolean("stop_arm", 0), std::memory_order_relaxed);

				double zero_angle;
				if(driveTable->GetBoolean("zero_navX", 0) != 0)
					zero_angle = (double)driveTable->GetNumber("zero_angle", 0);
				else
					zero_angle = -10000;
				navX_zero_.store(zero_angle, std::memory_order_relaxed);
			}

			last_nt_publish_time += ros::Duration(1.0 / nt_publish_rate);
		}

		//if (((last_joystick_publish_time + ros::Duration(1.0 / joystick_publish_rate)) < time_now_t) && 
		//	realtime_pub_joystick.trylock())
		if (realtime_pub_joystick.trylock())
		{
			realtime_pub_joystick.msg_.header.stamp = time_now_t;

			realtime_pub_joystick.msg_.rightStickY = joystick.GetRawAxis(5);
			realtime_pub_joystick.msg_.rightStickX = joystick.GetRawAxis(4);
			realtime_pub_joystick.msg_.leftStickY = joystick.GetRawAxis(1);
			realtime_pub_joystick.msg_.leftStickX = joystick.GetRawAxis(0);

			realtime_pub_joystick.msg_.leftTrigger = joystick.GetRawAxis(2);
			realtime_pub_joystick.msg_.rightTrigger = joystick.GetRawAxis(3);
			realtime_pub_joystick.msg_.buttonXButton = joystick.GetRawButton(3);
			realtime_pub_joystick.msg_.buttonXPress = joystick.GetRawButtonPressed(3);
			realtime_pub_joystick.msg_.buttonXRelease = joystick.GetRawButtonReleased(3);
			realtime_pub_joystick.msg_.buttonYButton = joystick.GetRawButton(4);
			realtime_pub_joystick.msg_.buttonYPress = joystick.GetRawButtonPressed(4);
			realtime_pub_joystick.msg_.buttonYRelease = joystick.GetRawButtonReleased(4);

			realtime_pub_joystick.msg_.bumperLeftButton = joystick.GetRawButton(5);
			realtime_pub_joystick.msg_.bumperLeftPress = joystick.GetRawButtonPressed(5);
			realtime_pub_joystick.msg_.bumperLeftRelease = joystick.GetRawButtonReleased(5);

			realtime_pub_joystick.msg_.bumperRightButton = joystick.GetRawButton(6);
			realtime_pub_joystick.msg_.bumperRightPress = joystick.GetRawButtonPressed(6);
			realtime_pub_joystick.msg_.bumperRightRelease = joystick.GetRawButtonReleased(6);

			realtime_pub_joystick.msg_.stickLeftButton = joystick.GetRawButton(9);
			realtime_pub_joystick.msg_.stickLeftPress = joystick.GetRawButtonPressed(9);
			realtime_pub_joystick.msg_.stickLeftRelease = joystick.GetRawButtonReleased(9);

			realtime_pub_joystick.msg_.stickRightButton = joystick.GetRawButton(10);
			realtime_pub_joystick.msg_.stickRightPress = joystick.GetRawButtonPressed(10);
			realtime_pub_joystick.msg_.stickRightRelease = joystick.GetRawButtonReleased(10);

			realtime_pub_joystick.msg_.buttonAButton = joystick.GetRawButton(1);
			realtime_pub_joystick.msg_.buttonAPress = joystick.GetRawButtonPressed(1);
			realtime_pub_joystick.msg_.buttonARelease = joystick.GetRawButtonReleased(1);
			realtime_pub_joystick.msg_.buttonBButton = joystick.GetRawButton(2);
			realtime_pub_joystick.msg_.buttonBPress = joystick.GetRawButtonPressed(2);
			realtime_pub_joystick.msg_.buttonBRelease = joystick.GetRawButtonReleased(2);
			realtime_pub_joystick.msg_.buttonBackButton = joystick.GetRawButton(7);
			realtime_pub_joystick.msg_.buttonBackPress = joystick.GetRawButtonPressed(7);
			realtime_pub_joystick.msg_.buttonBackRelease = joystick.GetRawButtonReleased(7);

			realtime_pub_joystick.msg_.buttonStartButton = joystick.GetRawButton(8);
			realtime_pub_joystick.msg_.buttonStartPress = joystick.GetRawButtonPressed(8);
			realtime_pub_joystick.msg_.buttonStartRelease = joystick.GetRawButtonReleased(8);
            
            /*-----------------------------------------------------------------------------*/
            /*-----------------------------------------------------------------------------*/
            /*-----------------------------------------------------------------------------*/
            /*****************************^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*******************/
            /*-----------------------------------------------------------------------------*/
            /*------------------------THIS IS JUST UNTIL ARM WORKS-------------------------*/
            /*-------------------------------DELETE THIS-----------------------------------*/
            /*****************************^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*******************/
            /*-----------------------------------------------------------------------------*/
            /*-----------------------------------------------------------------------------*/
            /*-----------------------------------------------------------------------------*/
			/*
			realtime_pub_joystick.msg_.buttonAButton = joystick.GetRawButton(8);
			realtime_pub_joystick.msg_.buttonAPress = joystick.GetRawButtonPressed(8);
			realtime_pub_joystick.msg_.buttonARelease = joystick.GetRawButtonReleased(8);

			realtime_pub_joystick.msg_.buttonXButton = joystick.GetRawButton(7);
			realtime_pub_joystick.msg_.buttonXPress = joystick.GetRawButtonPressed(7);
			realtime_pub_joystick.msg_.buttonXRelease = joystick.GetRawButtonReleased(7);

			realtime_pub_joystick.msg_.buttonBackButton = joystick.GetRawButton(3);
			realtime_pub_joystick.msg_.buttonBackPress = joystick.GetRawButtonPressed(3);
			realtime_pub_joystick.msg_.buttonBackRelease = joystick.GetRawButtonReleased(3);

			realtime_pub_joystick.msg_.buttonStartButton = joystick.GetRawButton(1);
			realtime_pub_joystick.msg_.buttonStartPress = joystick.GetRawButtonPressed(1);
			realtime_pub_joystick.msg_.buttonStartRelease = joystick.GetRawButtonReleased(1);
			*/
		
			switch (joystick.GetPOV(0))
			{
				default:{
						joystick_up_ = false;
						joystick_down_ = false;
						joystick_left_ = false;
						joystick_right_ = false;
						break;
					}
				case 0 :{
						joystick_up_ = true;
						joystick_down_ = false;
						joystick_left_ = false;
						joystick_right_ = false;
						break;
					} 
				case 45:{
						joystick_up_ = true;
						joystick_down_ = false;
						joystick_left_ = false;
						joystick_right_ = true;
						break;
					} 
				case 90:{
						joystick_up_ = false;
						joystick_down_ = false;
						joystick_left_ = false;
						joystick_right_ = true;
						break;
					} 
				case 135:{
						joystick_up_ = false;
						joystick_down_ = true;
						joystick_left_ = false;
						joystick_right_ = true;
						break;
					} 
				case 180:{
						joystick_up_ = false;
						joystick_down_ = true;
						joystick_left_ = false;
						joystick_right_ = false;
						break;
					} 
				case 225:{
						joystick_up_ = false;
						joystick_down_ = true;
						joystick_left_ = true;
						joystick_right_ = false;
						break;
					} 
				case 270:{
						joystick_up_ = false;
						joystick_down_ = false;
						joystick_left_ = true;
						joystick_right_ = false;
						break;
					} 
				case 315:{
						joystick_up_ = true;
						joystick_down_ = false;
						joystick_left_ = true;
						joystick_right_ = false;
						break;
					} 
			}
			
			realtime_pub_joystick.msg_.directionUpButton = joystick_up_;
			realtime_pub_joystick.msg_.directionUpPress = joystick_up_ > joystick_up_last_;
			realtime_pub_joystick.msg_.directionUpRelease = joystick_up_ > joystick_up_last_;
			
			realtime_pub_joystick.msg_.directionDownButton = joystick_down_;
			realtime_pub_joystick.msg_.directionDownPress = joystick_down_ > joystick_down_last_;
			realtime_pub_joystick.msg_.directionDownRelease = joystick_down_ > joystick_down_last_;

			realtime_pub_joystick.msg_.directionLeftButton = joystick_left_;
			realtime_pub_joystick.msg_.directionLeftPress = joystick_left_ > joystick_left_last_;
			realtime_pub_joystick.msg_.directionLeftRelease = joystick_left_ > joystick_left_last_;

			realtime_pub_joystick.msg_.directionRightButton = joystick_right_;
			realtime_pub_joystick.msg_.directionRightPress = joystick_right_ > joystick_right_last_;
			realtime_pub_joystick.msg_.directionRightRelease = joystick_right_ > joystick_right_last_;

			joystick_up_last_ = joystick_up_;
			joystick_down_last_ = joystick_down_;
			joystick_left_last_ = joystick_left_;
			joystick_right_last_ = joystick_right_;

			realtime_pub_joystick.unlockAndPublish();
		//	last_joystick_publish_time += ros::Duration(1.0 / joystick_publish_rate);
		}

		// Run at full speed until we see the game specific message.
		// This guaratees we react as quickly as possible to it.
		// After that is seen, slow down processing since there's nothing 
		// that changes that quickly in the data.
		if ((!game_specific_message_seen || (last_match_data_publish_time + ros::Duration(1.0 / match_data_publish_rate) < time_now_t)) && 
			realtime_pub_match_data.trylock())
		{
            //ROS_INFO("AA:%f", ros::Time::now().toSec());

			realtime_pub_match_data.msg_.matchTimeRemaining = DriverStation::GetInstance().GetMatchTime();

			const std::string game_specific_message = DriverStation::GetInstance().GetGameSpecificMessage();
			realtime_pub_match_data.msg_.allianceData = game_specific_message;
			realtime_pub_match_data.msg_.allianceColor = DriverStation::GetInstance().GetAlliance(); //returns int that corresponds to a DriverStation Alliance enum
			realtime_pub_match_data.msg_.driverStationLocation = DriverStation::GetInstance().GetLocation();
			realtime_pub_match_data.msg_.matchNumber = DriverStation::GetInstance().GetMatchNumber();
			realtime_pub_match_data.msg_.matchType = DriverStation::GetInstance().GetMatchType(); //returns int that corresponds to a DriverStation matchType enum

			const bool isEnabled = DriverStation::GetInstance().IsEnabled();
			realtime_pub_match_data.msg_.isEnabled = isEnabled;
			match_data_enabled_.store(isEnabled, std::memory_order_relaxed);

			realtime_pub_match_data.msg_.isDisabled = DriverStation::GetInstance().IsDisabled();
			realtime_pub_match_data.msg_.isAutonomous = DriverStation::GetInstance().IsAutonomous();

			realtime_pub_match_data.msg_.header.stamp = time_now_t;
			realtime_pub_match_data.unlockAndPublish();

			if (realtime_pub_match_data.msg_.isEnabled && (game_specific_message.length() > 0))
			{
				game_specific_message_seen = true;
				last_match_data_publish_time += ros::Duration(1.0 / match_data_publish_rate);
			}
			else
			{
				last_match_data_publish_time = ros::Time::now();
				game_specific_message_seen = false;
			}
		}
	}
}

void FRCRobotHWInterface::process_motion_profile_buffer_thread(double hz)
{
	return;
#if 0
	ros::Duration(3).sleep();	
	bool set_frame_period[num_can_talon_srxs_];
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
		set_frame_period[i] = false;

	ros::Rate rate(hz);
	while (ros::ok())
	{
		for (size_t i = 0; i < num_can_talon_srxs_; i++)
		{
			if ((*can_talons_mp_written_)[i].load(std::memory_order_relaxed))
			{
				const hardware_interface::TalonMode talon_mode = talon_state_[i].getTalonMode();
				const hardware_interface::MotionProfileStatus mp_status = talon_state_[i].getMotionProfileStatus();
				// Only write to non-follow, non-disabled talons that
				// have points to write from their top-level buffer
				//ROS_INFO_STREAM("top count: " << can_talons_[i]->GetMotionProfileTopLevelBufferCount());
				//ROS_WARN_STREAM("id: " << i << " top size: " << mp_status.topBufferCnt << " running: " << (*can_talons_mp_running_)[i].load(std::memory_order_relaxed));
				if (((talon_mode != hardware_interface::TalonMode_Follower) &&
				 /*can_talons_[i]->GetMotionProfileTopLevelBufferCount()*/ (mp_status.topBufferCnt 
				&& mp_status.btmBufferCnt < 127)) ||  
				(*can_talons_mp_running_)[i].load(std::memory_order_relaxed))
				{
					if (!set_frame_period[i])
					{
						can_talons_[i]->ChangeMotionControlFramePeriod(1000./hz); // 1000 to convert from sec to mSec
						talon_state_[i].setMotionControlFramePeriod(1000./hz);
						set_frame_period[i] = true;
					}
					// Only write if SW buffer has entries in it
					//ROS_INFO("needs to send points");
					(*can_talons_mp_writing_)[i].store(true, std::memory_order_relaxed);
					can_talons_[i]->ProcessMotionProfileBuffer();
				}
				else
				{
					(*can_talons_mp_writing_)[i].store(false, std::memory_order_relaxed);
				}
			}
		}
		rate.sleep();
	}
#endif
}
void FRCRobotHWInterface::custom_profile_set_talon(bool posMode, double setpoint, double fTerm, int joint_id, int pidSlot, bool zeroPos, double &pos_offset)
{


	
	const hardware_interface::FeedbackDevice encoder_feedback = talon_state_[joint_id].getEncoderFeedback();
	const int encoder_ticks_per_rotation = talon_state_[joint_id].getEncoderTicksPerRotation();
	const double conversion_factor = talon_state_[joint_id].getConversionFactor();

	const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position, joint_id) * conversion_factor;
	const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity, joint_id)* conversion_factor;
	
	if(zeroPos)
	{
		pos_offset = can_talons_[joint_id]->GetSelectedSensorPosition(pidIdx) * radians_scale;
		talon_state_[joint_id].setPosition(pos_offset);
	}
	//set talon
	ctre::phoenix::motorcontrol::ControlMode mode;
	hardware_interface::TalonMode mode_i;
	
	if(posMode) //Consider checking to see which point is closer
	{
		mode = ctre::phoenix::motorcontrol::ControlMode::Position;
		mode_i = hardware_interface::TalonMode_Position;
		setpoint += pos_offset;
		setpoint /= radians_scale; 

	}
	else
	{
		mode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
		mode_i = hardware_interface::TalonMode_Velocity;
		setpoint /= radians_per_second_scale; 
	}				
	can_talons_[joint_id]->Set(mode, setpoint, ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, fTerm); //TODO: unit conversion
	//ROS_INFO_STREAM("setpoint: " << setpoint << " fterm: " << fTerm << " id: " << joint_id << " offset " << pos_offset << " slot: " << pidSlot << " pos mode? " << posMode);
	talon_command_[joint_id].setMode(mode_i);
	talon_command_[joint_id].set(setpoint);
	talon_command_[joint_id].setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
	talon_command_[joint_id].setDemand1Value(fTerm);
	
	double command;
	hardware_interface::TalonMode in_mode;

	talon_command_[joint_id].newMode(in_mode);
	talon_command_[joint_id].commandChanged(command);

	hardware_interface::DemandType demand1_type_internal;
	double demand1_value;
	talon_command_[joint_id].demand1Changed(demand1_type_internal, demand1_value);

	talon_state_[joint_id].setDemand1Type(demand1_type_internal);
	talon_state_[joint_id].setDemand1Value(demand1_value);
				
	talon_state_[joint_id].setTalonMode(in_mode);
	talon_state_[joint_id].setSetpoint(command);

	talon_state_[joint_id].setNeutralOutput(false); // maybe make this a part of setSetpoint?


	talon_command_[joint_id].setPidfSlot(pidSlot);
	int dummy;
	if(talon_command_[joint_id].slotChanged(dummy))
	{
		can_talons_[joint_id]->SelectProfileSlot(pidSlot, timeoutMs);
		talon_state_[joint_id].setSlot(pidSlot);
	}

}
void FRCRobotHWInterface::custom_profile_thread(int joint_id)
{

	//TODO: somehow make this into a hw function and a base function in frc_robot_sim interface so the sim version is synced etc
	//I wonder how inefficient it is to have all of these threads 
	//running at the specified hz just copying to the status
	
	double time_start = ros::Time::now().toSec();
	int num_slots = 4; //Needs to be the same as the talon command interface and talon state interface
	hardware_interface::CustomProfileStatus status; //Status is also used to store info from last loop
	int points_run = 0;
	double pos_offset = 0;
	while (ros::ok())
	{
		ros::Rate rate(talon_command_[joint_id].getCustomProfileHz());
		bool run = talon_command_[joint_id].getCustomProfileRun();
		
		if(status.running && !run)
		{		
			std::vector<hardware_interface::CustomProfilePoint> empty_points;
			talon_command_[joint_id].overwriteCustomProfilePoints(empty_points, status.slotRunning);	
			//Right now we wipe everything if the profile is stopped
			//This could be changed to a pause type feature in which the first point has zeroPos set and the other
			//positions get shifted
			points_run = 0;
			pos_offset = 0;
		}
		if(run && !status.running || !run) 
		{
			time_start = ros::Time::now().toSec();
		}
		int slot = talon_command_[joint_id].getCustomProfileSlot();
	
		if(slot != status.slotRunning && run && status.running)
		{
			ROS_WARN("transitioned between two profile slots without any break between. Intended?");
			std::vector<hardware_interface::CustomProfilePoint> empty_points;
			talon_command_[joint_id].overwriteCustomProfilePoints(empty_points, status.slotRunning);	
			//Right now we wipe everything if the slots are flipped
			//Should try to be analagous to having a break between
			points_run = 0;
			pos_offset = 0;
			

		}
		status.slotRunning =  slot;	
		if(run)
		{
			auto profile = talon_command_[joint_id].getCustomProfilePoints(status.slotRunning);
			if(profile.size() == 0)
			{
				static int fail_flag = 0;
				if(fail_flag % 100 == 0)
				{
					ROS_ERROR("Tried to run custom profile with no points buffered");
				}
				//Potentially add more things to do if this exception is caught
				//Like maybe set talon to neutral mode or something
				fail_flag++;
				continue;
			}
			

			//TODO below isn't copying correct?
			auto times_by_point =  talon_command_[joint_id].getCustomProfileTime(status.slotRunning);
		
			int start = points_run - 1;
			if(start < 0) start = 0;
			int end;
			status.outOfPoints = true;
			double time_since_start = ros::Time::now().toSec() - time_start;
			for(; start < profile.size(); start++)
			{
				//Find the point just greater than time since start	
				if(times_by_point[start] > time_since_start)
				{
					status.outOfPoints = false;
					end = start;
					break;
				}
			}
			points_run = end -1;	
			if(points_run < 0) points_run = 0;
			auto next_slot = talon_command_[joint_id].getCustomProfileNextSlot();
			if(status.outOfPoints)
			{
				//If all points have been exhausted, just use the last point
				custom_profile_set_talon(profile.back().positionMode, profile.back().setpoint, profile.back().fTerm, joint_id, profile.back().pidSlot, profile.back().zeroPos, pos_offset);
				if((next_slot.size() > 0))
				{
					talon_command_[joint_id].setCustomProfileSlot(next_slot[0]);
					next_slot.erase(next_slot.begin());
					talon_command_[joint_id].setCustomProfileNextSlot(next_slot);
				}
			}
			else if(end ==0)
			{
				//If we are still on the first point,just use the first point
				custom_profile_set_talon(profile[0].positionMode, profile[0].setpoint, profile[0].fTerm, joint_id, profile[0].pidSlot, profile[0].zeroPos, pos_offset);
			}
			else
			{
				//Allows for mode flipping while in profile execution
				//We don't want to interpolate between positional and velocity setpoints
				if(profile[end].positionMode != profile[end-1].positionMode)
				{
					ROS_WARN("mid profile mode flip. If intended, Cooooooooollllll. If not, fix the code");
					custom_profile_set_talon(profile[end].positionMode, profile[end].setpoint, profile[end].fTerm, joint_id, profile[end].pidSlot, profile[end].zeroPos, pos_offset);
					// consider adding a check to see which is closer
				}
				else
				{
					//linear interpolation
					
					double setpoint = profile[end - 1].setpoint + (profile[end].setpoint - profile[end - 1].setpoint) / 
					(times_by_point[end] - times_by_point[end-1]) * (time_since_start - times_by_point[end-1]);

					
					double fTerm = profile[end - 1].fTerm + (profile[end].fTerm - profile[end - 1].fTerm) / 
					(times_by_point[end] - times_by_point[end-1]) * (time_since_start - times_by_point[end-1]);

					custom_profile_set_talon(profile[end].positionMode, setpoint, fTerm, joint_id, profile[end].pidSlot, profile[end-1].zeroPos, pos_offset);
				
				}

			}
		}
		else
		{
			status.outOfPoints = false;
		}
		for(int i = 0; i < num_slots; i++)
		{
			if(i == status.slotRunning)
            {
                status.remainingPoints[i] = talon_command_[joint_id].getCustomProfileCount(i) - points_run;
                if(talon_command_[joint_id].getCustomProfileTime(i).size() != 0)
                {
                    status.remainingTime = talon_command_[joint_id].getCustomProfileTime(i).back() - (ros::Time::now().toSec() - time_start);
                }
                else
                {
                    status.remainingTime = 0.0;
                }
            }
            else
            {
                status.remainingPoints[i] = talon_command_[joint_id].getCustomProfileCount(i);
            }

		}
		status.running = run; 
		talon_state_[joint_id].setCustomProfileStatus(status);
		rate.sleep();
	}
}
void FRCRobotHWInterface::init(void)
{
    ROS_ERROR("IN INIT");
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	FRCRobotInterface::init();

	// Make sure to initialize WPIlib code before creating
	// a CAN Talon object to avoid NIFPGA: Resource not initialized
	// errors? See https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
	hal_thread_ = std::thread(&FRCRobotHWInterface::hal_keepalive_thread, this);

	can_talons_mp_written_ = std::make_shared<std::vector<std::atomic<bool>>>(num_can_talon_srxs_);
	can_talons_mp_writing_ = std::make_shared<std::vector<std::atomic<bool>>>(num_can_talon_srxs_);
	can_talons_mp_running_ = std::make_shared<std::vector<std::atomic<bool>>>(num_can_talon_srxs_);
	custom_profile_threads_.resize(num_can_talon_srxs_);
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << can_talon_srx_names_[i] <<
							  " as CAN id " << can_talon_srx_can_ids_[i]);
		can_talons_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(can_talon_srx_can_ids_[i]));
		can_talons_[i]->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 50); // Make sure motor is stopped, use a long timeout just in case
		can_talons_[i]->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_10_MotionMagic, 10, 50); 
		//TODO: test above sketchy change
		// Make sure motor is stopped, use a long timeout just in case
		//safeTalonCall(can_talons_[i]->GetLastError(), "Initial Set(Disabled, 0)");
		//safeTalonCall(can_talons_[i]->ClearStickyFaults(timeoutMs), "ClearStickyFaults()");
		// TODO : if the talon doesn't initialize - maybe known
		// by -1 from firmware version read - somehow tag
		// the entry in can_talons_[] as uninitialized.
		// This probably should be a fatal error
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "\tTalon SRX firmware version " << can_talons_[i]->GetFirmwareVersion());
		// Clear sticky faults
		// safeTalonCall(can_talons_[1]->ClearStickyFaults(timeoutMs), "Clear sticky faults.");
		(*can_talons_mp_written_)[i].store(false, std::memory_order_relaxed);
		(*can_talons_mp_writing_)[i].store(false, std::memory_order_relaxed);
		(*can_talons_mp_running_)[i].store(false, std::memory_order_relaxed);
	
		
		custom_profile_threads_[i] = std::thread(&FRCRobotHWInterface::custom_profile_thread, this, i);

	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << nidec_brushless_names_[i] <<
							  " as PWM channel " << nidec_brushless_pwm_channels_[i] <<
							  " / DIO channel " << nidec_brushless_dio_channels_[i] <<
							  " invert " << nidec_brushless_inverts_[i]);

		nidec_brushlesses_.push_back(std::make_shared<frc::NidecBrushless>(nidec_brushless_pwm_channels_[i], nidec_brushless_dio_channels_[i]));
		nidec_brushlesses_[i]->SetInverted(nidec_brushless_inverts_[i]);
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << digital_input_names_[i] <<
							  " as Digital Input " << digital_input_dio_channels_[i] <<
							  " invert " << digital_input_inverts_[i]);

		digital_inputs_.push_back(std::make_shared<frc::DigitalInput>(digital_input_dio_channels_[i]));
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << digital_output_names_[i] <<
							  " as Digital Output " << digital_output_dio_channels_[i] <<
							  " invert " << digital_output_inverts_[i]);

		digital_outputs_.push_back(std::make_shared<frc::DigitalOutput>(digital_output_dio_channels_[i]));
	}
	for (size_t i = 0; i < num_pwm_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << pwm_names_[i] <<
							  " as Digitial Output " << pwm_pwm_channels_[i] <<
							  " invert " << pwm_inverts_[i]);

		PWMs_.push_back(std::make_shared<frc::SafePWM>(pwm_pwm_channels_[i]));
		PWMs_[i]->SetSafetyEnabled(true);
	}
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << solenoid_names_[i] <<
							  " as Solenoid " << solenoid_ids_[i]
							  << " with pcm " << solenoid_pcms_[i]);

		solenoids_.push_back(std::make_shared<frc::Solenoid>(solenoid_pcms_[i], solenoid_ids_[i]));
	}
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << double_solenoid_names_[i] <<
							  " as Double Solenoid forward " << double_solenoid_forward_ids_[i] <<
							  " reverse " << double_solenoid_reverse_ids_[i]
							  << " with pcm " << double_solenoid_pcms_[i]);

		double_solenoids_.push_back(std::make_shared<frc::DoubleSolenoid>(double_solenoid_pcms_[i], double_solenoid_forward_ids_[i], double_solenoid_reverse_ids_[i]));
	}

	//RIGHT NOW THIS WILL ONLY WORK IF THERE IS ONLY ONE NAVX INSTANTIATED
	for(size_t i = 0; i < num_navX_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
				"Loading joint " << i << "=" << navX_names_[i] <<
				" as navX id " << navX_ids_[i]);
		//TODO: fix how we use ids

		navXs_.push_back(std::make_shared<AHRS>(SPI::Port::kMXP));

		// This is a guess so TODO : get better estimates
		imu_orientation_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		imu_angular_velocity_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
	   	imu_linear_acceleration_covariances_[i] ={0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
	}
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << analog_input_names_[i] <<
							  " as Analog Input " << analog_input_analog_channels_[i]);
		analog_inputs_.push_back(std::make_shared<frc::AnalogInput>(analog_input_analog_channels_[i]));
	}
	for (size_t i = 0; i < num_compressors_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << compressor_names_[i] <<
							  " as Compressor with pcm " << compressor_pcm_ids_[i]);

		compressors_.push_back(std::make_shared<frc::Compressor>(compressor_pcm_ids_[i]));
	}

	stop_arm_  = false;
	override_arm_limits_ = false;
	cube_state_ = false;
	disable_compressor_ = false;
	navX_zero_ = -10000;
	navX_angle_ = 0;
	pressure_ = 0;
	match_data_enabled_ = false;

	for(size_t i = 0; i < num_dummy_joints_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading dummy joint " << i << "=" << dummy_joint_names_[i]);

	//HAL_InitializePDP(0,0);
	//int32_t status = 0;
	//HAL_ResetPDPTotalEnergy(0, &status);
	//HAL_ClearPDPStickyFaults(0, &status);
	pdp_joint_.ClearStickyFaults();
	pdp_joint_.ResetTotalEnergy();

	motion_profile_thread_ = std::thread(&FRCRobotHWInterface::process_motion_profile_buffer_thread, this, 100.);
	ROS_INFO_NAMED("frcrobot_hw_interface", "FRCRobotHWInterface Ready.");
}

void FRCRobotHWInterface::read(ros::Duration &/*elapsed_time*/)
{
	//return;
	//
#if 0		
	const int talon_updates_to_skip = 2;
	static int talon_skip_counter = 0;
	static int next_talon_to_read = 0;
	size_t read_this_talon = std::numeric_limits<size_t>::max();
	
	// Try to load-balance reading from talons
	// Loop through and read non-critical data from only one
	// joint per read() pass.  Only do that read every
	// talon_updates_to_skip times through the loop.
	if (++talon_skip_counter == talon_updates_to_skip)
	{
		talon_skip_counter = 0;
		read_this_talon = next_talon_to_read;
		if(num_can_talon_srxs_!= 0)
		{
			next_talon_to_read = (next_talon_to_read + 1) % num_can_talon_srxs_;
		}
	}
#endif
	bool profile_is_live = false;
	bool writing_points = false;
	for(std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		if((*can_talons_mp_running_)[joint_id].load(std::memory_order_relaxed))
		{
			profile_is_live = true;
			break;	
		} 
	}
	for(std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		if((*can_talons_mp_writing_)[joint_id].load(std::memory_order_relaxed))
		{
			writing_points = true;
			break;	
		} 
	}
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		auto &ts = talon_state_[joint_id];
		auto &talon = can_talons_[joint_id];
		if (!talon) // skip unintialized Talons
			continue;
		if (ts.getCANID() == 31 || ts.getCANID() == 32)
			continue;

		// read position and velocity from can_talons_[joint_id]
		// convert to whatever units make sense
		const hardware_interface::FeedbackDevice encoder_feedback = ts.getEncoderFeedback();
		const hardware_interface::TalonMode talon_mode = ts.getTalonMode();
		if (talon_mode == hardware_interface::TalonMode_Follower)
			continue;

		/*	
		if((*can_talons_mp_running_)[joint_id].load(std::memory_order_relaxed))
		{
			ROS_INFO_STREAM("running");
		}
		if((*can_talons_mp_writing_)[joint_id].load(std::memory_order_relaxed))
		{
			ROS_INFO_STREAM("writing");
		}
		if((*can_talons_mp_written_)[joint_id].load(std::memory_order_relaxed))
		{
			ROS_INFO_STREAM("written");
		}
		*/
		const int encoder_ticks_per_rotation = ts.getEncoderTicksPerRotation();
		const double conversion_factor = ts.getConversionFactor();

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position, joint_id) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity, joint_id)* conversion_factor;

		if(ts.getCANID() == 51)
		{
			auto sensor_collection = talon->GetSensorCollection();
			ts.setForwardLimitSwitch(sensor_collection.IsFwdLimitSwitchClosed());
			ts.setReverseLimitSwitch(sensor_collection.IsRevLimitSwitchClosed());
		}
		if(profile_is_live)
		{
			if(ts.getCANID() == 51 || ts.getCANID() == 41) //All we care about are the arm and lift
			{
				const double position = talon->GetSelectedSensorPosition(pidIdx) * radians_scale;
				safeTalonCall(talon->GetLastError(), "GetSelectedSensorPosition");
				ts.setPosition(position);
			}
			continue;
		}
		if(writing_points)
		{
			if(ts.getCANID() == 51 || ts.getCANID() == 41) //All we care about are the arm and lift
			{
				const double position = talon->GetSelectedSensorPosition(pidIdx) * radians_scale;
				safeTalonCall(talon->GetLastError(), "GetSelectedSensorPosition");
				ts.setPosition(position);
			}
			
			if(ts.getCANID() > 30) continue;


			ctre::phoenix::motion::MotionProfileStatus talon_status;
			safeTalonCall(talon->GetMotionProfileStatus(talon_status), "GetMotionProfileStatus");

			hardware_interface::MotionProfileStatus internal_status;
			internal_status.topBufferRem = talon_status.topBufferRem;
			internal_status.topBufferCnt = talon_status.topBufferCnt;
			internal_status.btmBufferCnt = talon_status.btmBufferCnt;
			internal_status.hasUnderrun = talon_status.hasUnderrun;
			internal_status.isUnderrun = talon_status.isUnderrun;
			internal_status.activePointValid = talon_status.activePointValid;
			internal_status.isLast = talon_status.isLast;
			internal_status.profileSlotSelect0 = talon_status.profileSlotSelect0;
			internal_status.profileSlotSelect1 = talon_status.profileSlotSelect1;
			internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);
			internal_status.timeDurMs = talon_status.timeDurMs;
			ts.setMotionProfileStatus(internal_status);
			continue;
		}
		else if(ts.getCANID() < 30 && (*can_talons_mp_written_)[joint_id].load(std::memory_order_relaxed))
		{
			
			ctre::phoenix::motion::MotionProfileStatus talon_status;
			safeTalonCall(talon->GetMotionProfileStatus(talon_status), "GetMotionProfileStatus");

			hardware_interface::MotionProfileStatus internal_status;
			internal_status.topBufferRem = talon_status.topBufferRem;
			internal_status.topBufferCnt = talon_status.topBufferCnt;
			internal_status.btmBufferCnt = talon_status.btmBufferCnt;
			internal_status.hasUnderrun = talon_status.hasUnderrun;
			internal_status.isUnderrun = talon_status.isUnderrun;
			internal_status.activePointValid = talon_status.activePointValid;
			internal_status.isLast = talon_status.isLast;
			internal_status.profileSlotSelect0 = talon_status.profileSlotSelect0;
			internal_status.profileSlotSelect1 = talon_status.profileSlotSelect1;
			internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);
			internal_status.timeDurMs = talon_status.timeDurMs;
			ts.setMotionProfileStatus(internal_status);
		}  	
		const double position = talon->GetSelectedSensorPosition(pidIdx) * radians_scale;
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorPosition");
		ts.setPosition(position);

		const double speed = talon->GetSelectedSensorVelocity(pidIdx) * radians_per_second_scale;
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorVelocity");
		ts.setSpeed(speed);

		//top level buffer has capacity of 4096
		//ROS_INFO_STREAM("num rem : " << talon_status.topBufferRem);

		//const double output_current = talon->GetOutputCurrent();
		//safeTalonCall(talon->GetLastError(), "GetOutputCurrent");
		//ts.setOutputCurrent(output_current);

	//	if (read_this_talon == joint_id)
		{
			//const double bus_voltage = talon->GetBusVoltage();
			//safeTalonCall(talon->GetLastError(), "GetBusVoltage");
			//ts.setBusVoltage(bus_voltage);

			//const double motor_output_percent = talon->GetMotorOutputPercent();
			//safeTalonCall(talon->GetLastError(), "GetMotorOutputPercent");
			//ts.setMotorOutputPercent(motor_output_percent);

			//const double output_voltage = talon->GetMotorOutputVoltage();
			//safeTalonCall(talon->GetLastError(), "GetMotorOutputVoltage");
			//ts.setOutputVoltage(output_voltage);

			//const double temperature = talon->GetTemperature(); //returns in Celsius
			//safeTalonCall(talon->GetLastError(), "GetTemperature");
			//ts.setTemperature(temperature);

			//closed-loop
			if ((talon_mode == hardware_interface::TalonMode_Position) ||
					(talon_mode == hardware_interface::TalonMode_Velocity) ||
					(talon_mode == hardware_interface::TalonMode_Current ) ||
					(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
					(talon_mode == hardware_interface::TalonMode_MotionMagic))
			{
				//const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode, joint_id)* conversion_factor;
				
				//const double closed_loop_error = talon->GetClosedLoopError(pidIdx) * closed_loop_scale;
				//safeTalonCall(talon->GetLastError(), "GetClosedLoopError");
				//ts.setClosedLoopError(closed_loop_error);
				//const double integral_accumulator = talon->GetIntegralAccumulator(pidIdx) * closed_loop_scale;
				//safeTalonCall(talon->GetLastError(), "GetIntegralAccumulator");
				//ts.setIntegralAccumulator(integral_accumulator);

				//const double error_derivative = talon->GetErrorDerivative(pidIdx) * closed_loop_scale;
				//safeTalonCall(talon->GetLastError(), "GetErrorDerivative");
				//ts.setErrorDerivative(error_derivative);

				//const double closed_loop_target = talon->GetClosedLoopTarget(pidIdx) * closed_loop_scale;
				//safeTalonCall(talon->GetLastError(), "GetClosedLoopTarget");
				//ts.setClosedLoopTarget(closed_loop_target);
			}

			if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
					(talon_mode == hardware_interface::TalonMode_MotionMagic))
			{
				//const double active_trajectory_position = talon->GetActiveTrajectoryPosition() * radians_scale;
				//safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryPosition");
				//ts.setActiveTrajectoryPosition(active_trajectory_position);
				//const double active_trajectory_velocity = talon->GetActiveTrajectoryVelocity() * radians_per_second_scale;
				//safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryVelocity");
				//ts.setActiveTrajectoryVelocity(active_trajectory_velocity);
				//const double active_trajectory_heading = talon->GetActiveTrajectoryHeading() * 2.*M_PI / 360.; //returns in degrees
				//safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryHeading");
				//ts.setActiveTrajectoryHeading(active_trajectory_heading);
				//ts.setMotionProfileTopLevelBufferCount(talon->GetMotionProfileTopLevelBufferCount());

				//safeTalonCall(talon->GetLastError(), "IsMotionProfileTopLevelBufferFull");

			}

			//ctre::phoenix::motorcontrol::Faults faults;
			//safeTalonCall(talon->GetFaults(faults), "GetFaults");
			//ts.setFaults(faults.ToBitfield());

			// Grab limit switch and softlimit here
			//auto sensor_collection = talon->GetSensorCollection();
			//ts.setForwardLimitSwitch(sensor_collection.IsFwdLimitSwitchClosed());
			//ts.setReverseLimitSwitch(sensor_collection.IsRevLimitSwitchClosed());

			//ts.setForwardSoftlimitHit(faults.ForwardSoftLimit);
			//ts.setReverseSoftlimitHit(faults.ReverseSoftLimit);

			//ctre::phoenix::motorcontrol::StickyFaults sticky_faults;
			//safeTalonCall(talon->GetStickyFaults(sticky_faults), "GetStickyFaults");
			//ts.setStickyFAults(sticky_faults.ToBitfield());
		}
	}
		
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		brushless_vel_[i] = nidec_brushlesses_[i]->Get();
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		//State should really be a bool - but we're stuck using
		//ROS control code which thinks everything to and from
		//hardware are doubles
		digital_input_state_[i] = (digital_inputs_[i]->Get()^digital_input_inverts_[i]) ? 1 : 0;
	}
#if 0
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		digital_output_state_[i] = (digital_outputs_[i]->Get()^digital_output_inverts_[i]) ? 1 : 0;
		//State should really be a bool
		//This isn't strictly neccesary, it just reads what the DIO is currently set to
	}
#endif
#if 0
	for (size_t i = 0; i < num_pwm_; i++)
	{
		// Just reflect state of output in status
		//pwm_state_[i] = PWMs_[i]->GetSpeed();
	}
#endif
#if 0
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		solenoid_state_[i] = solenoids_[i]->Get();
	}
#endif
#if 0
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		double_solenoid_state_[i] = double_solenoids_[i]->Get();
	}
#endif
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		analog_input_state_[i] = analog_inputs_[i]->GetValue() *analog_input_a_[i] + analog_input_b_[i];
		if(analog_input_names_[i] == "analog_pressure_sensor")
		{
			pressure_.store(analog_input_state_[i], std::memory_order_relaxed);
		}
	}
 	//navX read here
	for (size_t i = 0; i < num_navX_; i++)
	{
		// TODO : double check we're reading
		// the correct data

		// navXs_[i]->GetFusedHeading();
		// navXs_[i]->GetPitch();
		// navXs_[i]->GetRoll();

		// TODO : Fill in imu_angular_velocity[i][]

		//navXs_[i]->IsCalibrating();
		//navXs_[i]->IsConnected();
		//navXs_[i]->GetLastSensorTimestamp();
		//
		imu_linear_accelerations_[i][0] = navXs_[i]->GetWorldLinearAccelX();
		imu_linear_accelerations_[i][1] = navXs_[i]->GetWorldLinearAccelY();
		imu_linear_accelerations_[i][2] = navXs_[i]->GetWorldLinearAccelZ();

		//navXs_[i]->IsMoving();
		//navXs_[i]->IsRotating();
		//navXs_[i]->IsMagneticDisturbance();
		//navXs_[i]->IsMagnetometerCalibrated();
		//
		tf2::Quaternion tempQ;
		if(i == 0)
		{
			const double navX_zero = navX_zero_.load(std::memory_order_relaxed);
			if(navX_zero != -10000)
				offset_navX_[i] = navX_zero - navXs_[i]->GetFusedHeading() / 360. * 2. * M_PI;

			navX_angle_.store(navXs_[i]->GetFusedHeading() / 360. * 2. * M_PI + offset_navX_[i], std::memory_order_relaxed);
		}
		tempQ.setRPY(navXs_[i]->GetRoll() / -360 * 2 * M_PI, navXs_[i]->GetPitch() / -360 * 2 * M_PI, navXs_[i]->GetFusedHeading() / 360 * 2 * M_PI + offset_navX_[i]  );

		imu_orientations_[i][3] = tempQ.w();
		imu_orientations_[i][0] = tempQ.x();
		imu_orientations_[i][1] = tempQ.y();
		imu_orientations_[i][2] = tempQ.z();

		imu_angular_velocities_[i][0] = navXs_[i]->GetVelocityX();
		imu_angular_velocities_[i][1] = navXs_[i]->GetVelocityY();
		imu_angular_velocities_[i][2] = navXs_[i]->GetVelocityZ();

		//navXs_[i]->GetDisplacementX();
		//navXs_[i]->GetDisplacementY();
		//navXs_[i]->GetDisplacementZ();
		//navXs_[i]->GetAngle(); //continous
		//TODO: add setter functions
		
		navX_state_[i] = offset_navX_[i];
	}
	
	/*for (size_t i = 0; i < num_compressors_; i++)
	{
		compressor_state_[i] = compressors_[i]->GetCompressorCurrent();
	}*/
	if(!profile_is_live && !writing_points)
	{
		//read info from the PDP hardware
		pdp_state_.setVoltage(pdp_joint_.GetVoltage());
		pdp_state_.setTemperature(pdp_joint_.GetTemperature());
		pdp_state_.setTotalCurrent(pdp_joint_.GetTotalCurrent());
		pdp_state_.setTotalPower(pdp_joint_.GetTotalPower());
		pdp_state_.setTotalEnergy(pdp_joint_.GetTotalEnergy());
		for(int channel = 0; channel <= 15; channel++)
		{
			pdp_state_.setCurrent(pdp_joint_.GetCurrent(channel), channel);
		}
	}

}

double FRCRobotHWInterface::getConversionFactor(int encoder_ticks_per_rotation,
						hardware_interface::FeedbackDevice encoder_feedback,
						hardware_interface::TalonMode talon_mode,
						int joint_id)
{
	if(talon_mode == hardware_interface::TalonMode_Position)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2 * M_PI / encoder_ticks_per_rotation;
			case hardware_interface::FeedbackDevice_Analog:
				return 2 * M_PI / 1024;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
				return 1.;
		}
	}
	else if(talon_mode == hardware_interface::TalonMode_Velocity)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2 * M_PI / encoder_ticks_per_rotation / .1;
			case hardware_interface::FeedbackDevice_Analog:
				return 2 * M_PI / 1024 / .1;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
				return 1.;
		}
	}
	else
	{
		//ROS_WARN_STREAM("Unable to convert closed loop units.");
		return 1.;
	}
}

bool FRCRobotHWInterface::safeTalonCall(ctre::phoenix::ErrorCode error_code, const std::string &talon_method_name)
{
	std::string error_name;
	switch (error_code)
	{
		case ctre::phoenix::OK :
			return true; // Yay us!

		case ctre::phoenix::CAN_MSG_STALE :
			error_name = "CAN_MSG_STALE/CAN_TX_FULL/TxFailed";
			break;
		case ctre::phoenix::InvalidParamValue :
			error_name = "InvalidParamValue/CAN_INVALID_PARAM";
			break;

		case ctre::phoenix::RxTimeout :
			error_name = "RxTimeout/CAN_MSG_NOT_FOUND";
			break;
		case ctre::phoenix::TxTimeout :
			error_name = "TxTimeout/CAN_NO_MORE_TX_JOBS";
			break;
		case ctre::phoenix::UnexpectedArbId :
			error_name = "UnexpectedArbId/CAN_NO_SESSIONS_AVAIL";
			break;
		case ctre::phoenix::BufferFull :
			error_name = "BufferFull/CAN_OVERFLOW";
			break;
		case ctre::phoenix::SensorNotPresent :
			error_name = "SensorNotPresent";
			break;
		case ctre::phoenix::FirmwareTooOld :
			error_name = "FirmwareTooOld";
			break;
		case ctre::phoenix::CouldNotChangePeriod :
			error_name = "CouldNotChangePeriod";
			break;

		case ctre::phoenix::GENERAL_ERROR :
			error_name = "GENERAL_ERROR";
			break;

		case ctre::phoenix::SIG_NOT_UPDATED :
			error_name = "SIG_NOT_UPDATED";
			break;
		case ctre::phoenix::NotAllPIDValuesUpdated :
			error_name = "NotAllPIDValuesUpdated";
			break;

		case ctre::phoenix::GEN_PORT_ERROR :
			error_name = "GEN_PORT_ERROR";
			break;
		case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH :
			error_name = "PORT_MODULE_TYPE_MISMATCH";
			break;

		case ctre::phoenix::GEN_MODULE_ERROR :
			error_name = "GEN_MODULE_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR :
			error_name = "MODULE_NOT_INIT_SET_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR :
			error_name = "MODULE_NOT_INIT_GET_ERROR";
			break;

		case ctre::phoenix::WheelRadiusTooSmall :
			error_name = "WheelRadiusTooSmall";
			break;
		case ctre::phoenix::TicksPerRevZero :
			error_name = "TicksPerRevZero";
			break;
		case ctre::phoenix::DistanceBetweenWheelsTooSmall :
			error_name = "DistanceBetweenWheelsTooSmall";
			break;
		case ctre::phoenix::GainsAreNotSet :
			error_name = "GainsAreNotSet";
			break;
		case ctre::phoenix::IncompatibleMode :
			error_name = "IncompatibleMode";
			break;
		case ctre::phoenix::InvalidHandle :
			error_name = "InvalidHandle";
			break;

		case ctre::phoenix::FeatureRequiresHigherFirm:
			error_name = "FeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::TalonFeatureRequiresHigherFirm:
			error_name = "TalonFeatureRequiresHigherFirm";
			break;
 
		case ctre::phoenix::PulseWidthSensorNotPresent :
			error_name = "PulseWidthSensorNotPresent";
			break;
		case ctre::phoenix::GeneralWarning :
			error_name = "GeneralWarning";
			break;
		case ctre::phoenix::FeatureNotSupported :
			error_name = "FeatureNotSupported";
			break;
		case ctre::phoenix::NotImplemented :
			error_name = "NotImplemented";
			break;
		case ctre::phoenix::FirmVersionCouldNotBeRetrieved :
			error_name = "FirmVersionCouldNotBeRetrieved";
			break;
		case ctre::phoenix::FeaturesNotAvailableYet :
			error_name = "FeaturesNotAvailableYet";
			break;
		case ctre::phoenix::ControlModeNotValid :
			error_name = "ControlModeNotValid";
			break;

		case ctre::phoenix::ControlModeNotSupportedYet :
			error_name = "case";
			break;
		case ctre::phoenix::CascadedPIDNotSupporteYet:
			error_name = "CascadedPIDNotSupporteYet/AuxiliaryPIDNotSupportedYet";
			break;
		case ctre::phoenix::RemoteSensorsNotSupportedYet:
			error_name = "RemoteSensorsNotSupportedYet";
			break;
		case ctre::phoenix::MotProfFirmThreshold:
			error_name = "MotProfFirmThreshold";
			break;
		case ctre::phoenix::MotProfFirmThreshold2:
			error_name = "MotProfFirmThreshold2";
			break;

		default:
			{
				std::stringstream s;
				s << "Unknown Talon error " << error_code;
				error_name = s.str();
				break;
			}

	}
	//ROS_ERROR_STREAM("Error calling " << talon_method_name << " : " << error_name);
	return false;
}

void FRCRobotHWInterface::write(ros::Duration &elapsed_time)
{
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		//TODO : skip over most or all of this if the talon is in follower mode
		//       Only do the Set() call and then
		//       never do anything else?  Need to make sure things like inverts
		//       and so on are copied from the talon it is following - RG inverts shouldn't
		//       be copied, we may need to run a slave inverted relative to master
		//
		// Save some typing by making references to commonly
		// used variables
		auto &talon = can_talons_[joint_id];

		if (!talon) // skip unintialized Talons
			continue;



		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];
		
		if(tc.getCustomProfileRun())
			continue; //Don't mess with talons running in custom profile mode

		hardware_interface::FeedbackDevice internal_feedback_device;
		double feedback_coefficient;

		ctre::phoenix::motorcontrol::FeedbackDevice talon_feedback_device;
		if (tc.encoderFeedbackChanged(internal_feedback_device, feedback_coefficient) &&
			convertFeedbackDevice(internal_feedback_device, talon_feedback_device))
		{
			//ROS_WARN("feedback");
			safeTalonCall(talon->ConfigSelectedFeedbackSensor(talon_feedback_device, pidIdx, timeoutMs),"ConfigSelectedFeedbackSensor");
			safeTalonCall(talon->ConfigSelectedFeedbackCoefficient(feedback_coefficient, pidIdx, timeoutMs),"ConfigSelectedFeedbackCoefficient");
 			ts.setEncoderFeedback(internal_feedback_device);
			ts.setFeedbackCoefficient(feedback_coefficient);
		}

		const hardware_interface::TalonMode talon_mode = ts.getTalonMode();
		const int encoder_ticks_per_rotation = tc.getEncoderTicksPerRotation();
		ts.setEncoderTicksPerRotation(encoder_ticks_per_rotation);

		double conversion_factor;
		if (tc.conversionFactorChanged(conversion_factor))
			ts.setConversionFactor(conversion_factor);

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Position, joint_id) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Velocity, joint_id) * conversion_factor;
		const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, talon_mode, joint_id) * conversion_factor;

		bool close_loop_mode = false;
		bool motion_profile_mode = false;

		if ((talon_mode == hardware_interface::TalonMode_Position) ||
		    (talon_mode == hardware_interface::TalonMode_Velocity) ||
		    (talon_mode == hardware_interface::TalonMode_Current ))
		{
			close_loop_mode = true;
		}
		else if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			     (talon_mode == hardware_interface::TalonMode_MotionMagic))
		{
			close_loop_mode = true;
			motion_profile_mode = true;
		}

		if (close_loop_mode)
		{
			int slot;
			const bool slot_changed = tc.slotChanged(slot);
			
			double p;
			double i;
			double d;
			double f;
			int    iz;
			int    allowable_closed_loop_error;
			double max_integral_accumulator;
			double closed_loop_peak_output;
			int    closed_loop_period;
 
			if (tc.pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, closed_loop_peak_output, closed_loop_period, slot))
			{
				//ROS_WARN("PIDF");
				safeTalonCall(talon->Config_kP(slot, p, timeoutMs),"Config_kP");
				safeTalonCall(talon->Config_kI(slot, i, timeoutMs),"Config_kI");
				safeTalonCall(talon->Config_kD(slot, d, timeoutMs),"Config_kD");
				safeTalonCall(talon->Config_kF(slot, f, timeoutMs),"Config_kF");
				safeTalonCall(talon->Config_IntegralZone(slot, iz, timeoutMs),"Config_IntegralZone");
				// TODO : Scale these two?
				safeTalonCall(talon->ConfigAllowableClosedloopError(slot, allowable_closed_loop_error, timeoutMs),"ConfigAllowableClosedloopError");
				safeTalonCall(talon->ConfigMaxIntegralAccumulator(slot, max_integral_accumulator, timeoutMs),"ConfigMaxIntegralAccumulator");
				safeTalonCall(talon->ConfigClosedLoopPeakOutput(slot, closed_loop_peak_output, timeoutMs),"ConfigClosedLoopPeakOutput");
				safeTalonCall(talon->ConfigClosedLoopPeriod(slot, closed_loop_period, timeoutMs),"ConfigClosedLoopPeriod");

				ts.setPidfP(p, slot);
				ts.setPidfI(i, slot);
				ts.setPidfD(d, slot);
				ts.setPidfF(f, slot);
				ts.setPidfIzone(iz, slot);
				ts.setAllowableClosedLoopError(allowable_closed_loop_error, slot);
				ts.setMaxIntegralAccumulator(max_integral_accumulator, slot);
				ts.setClosedLoopPeakOutput(closed_loop_peak_output, slot);
				ts.setClosedLoopPeriod(closed_loop_period, slot);

				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" PIDF slot " << slot << " config values");
			}
 
			bool aux_pid_polarity;
			if (tc.auxPidPolarityChanged(aux_pid_polarity))
			{
				safeTalonCall(talon->ConfigAuxPIDPolarity(aux_pid_polarity, timeoutMs), "ConfigAuxPIDPolarity");
				ts.setAuxPidPolarity(aux_pid_polarity);
			}

			if (slot_changed)
			{
				//ROS_WARN("slot");
				ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF slot to " << slot << std::endl);

				safeTalonCall(talon->SelectProfileSlot(slot, pidIdx),"SelectProfileSlot");
				ts.setSlot(slot);
			}
		}

		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			//ROS_WARN("invvert");
			talon->SetInverted(invert);
			safeTalonCall(talon->GetLastError(), "SetInverted");
			talon->SetSensorPhase(sensor_phase);
			safeTalonCall(talon->GetLastError(), "SetSensorPhase");
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" invert / phase");
		}

		hardware_interface::NeutralMode neutral_mode;
		ctre::phoenix::motorcontrol::NeutralMode ctre_neutral_mode;
		if (tc.neutralModeChanged(neutral_mode) &&
			convertNeutralMode(neutral_mode, ctre_neutral_mode))
		{
			//ROS_WARN("neutral2");
			talon->SetNeutralMode(ctre_neutral_mode);
			safeTalonCall(talon->GetLastError(), "SetNeutralMode");
			ts.setNeutralMode(neutral_mode);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral mode");
		}

		if (tc.neutralOutputChanged())
		{
			//ROS_WARN("neutral");
			talon->NeutralOutput();
			safeTalonCall(talon->GetLastError(), "NeutralOutput");
			ts.setNeutralOutput(true);
			ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral output");
		}

		// Force I accumulator to zero so the robot isn't enabled
		// with a huge I term driving the controlled motor
		if (!match_data_enabled_.load(std::memory_order_relaxed))
			safeTalonCall(talon->SetIntegralAccumulator(0, pidIdx, timeoutMs), "SetIntegralAccumulator");

		double iaccum;
		if (close_loop_mode && tc.integralAccumulatorChanged(iaccum))
		{
			//ROS_WARN("iaccum");
			safeTalonCall(talon->SetIntegralAccumulator(iaccum / closed_loop_scale, pidIdx, timeoutMs), "SetIntegralAccumulator");
			//The units on this aren't really right

			// Do not set talon state - this changes
			// dynamically so read it in read() above instead
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
			//ROS_WARN("output shape");
			safeTalonCall(talon->ConfigOpenloopRamp(open_loop_ramp, timeoutMs),"ConfigOpenloopRamp");
			safeTalonCall(talon->ConfigClosedloopRamp(closed_loop_ramp, timeoutMs),"ConfigClosedloopRamp");
			safeTalonCall(talon->ConfigPeakOutputForward(peak_output_forward, timeoutMs),"ConfigPeakOutputForward");          // 100
			safeTalonCall(talon->ConfigPeakOutputReverse(peak_output_reverse, timeoutMs),"ConfigPeakOutputReverse");          // -100
			safeTalonCall(talon->ConfigNominalOutputForward(nominal_output_forward, timeoutMs),"ConfigNominalOutputForward"); // 0
			safeTalonCall(talon->ConfigNominalOutputReverse(nominal_output_reverse, timeoutMs),"ConfigNominalOutputReverse"); // 0
			safeTalonCall(talon->ConfigNeutralDeadband(neutral_deadband, timeoutMs),"ConfigNeutralDeadband");                 // 0

			ts.setOpenloopRamp(open_loop_ramp);
			ts.setClosedloopRamp(closed_loop_ramp);
			ts.setPeakOutputForward(peak_output_forward);
			ts.setPeakOutputReverse(peak_output_reverse);
			ts.setNominalOutputForward(nominal_output_forward);
			ts.setNominalOutputReverse(nominal_output_reverse);
			ts.setNeutralDeadband(neutral_deadband);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" output shaping");
		}
		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.voltageCompensationChanged(v_c_saturation,
										  v_measurement_filter,
										  v_c_enable))
		{
			//ROS_WARN("volt comp");
			safeTalonCall(talon->ConfigVoltageCompSaturation(v_c_saturation, timeoutMs),"ConfigVoltageCompSaturation");
			safeTalonCall(talon->ConfigVoltageMeasurementFilter(v_measurement_filter, timeoutMs),"ConfigVoltageMeasurementFilter");
			talon->EnableVoltageCompensation(v_c_enable);
			safeTalonCall(talon->GetLastError(), "EnableVoltageCompensation");

			ts.setVoltageCompensationSaturation(v_c_saturation);
			ts.setVoltageMeasurementFilter(v_measurement_filter);
			ts.setVoltageCompensationEnable(v_c_enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" voltage compensation");
		}

		hardware_interface::VelocityMeasurementPeriod internal_v_m_period;
		ctre::phoenix::motorcontrol::VelocityMeasPeriod phoenix_v_m_period;
		int v_m_window;

		if (tc.velocityMeasurementChanged(internal_v_m_period, v_m_window) &&
			convertVelocityMeasurementPeriod(internal_v_m_period, phoenix_v_m_period))
		{
			safeTalonCall(talon->ConfigVelocityMeasurementPeriod(phoenix_v_m_period, timeoutMs),"ConfigVelocityMeasurementPeriod");
			safeTalonCall(talon->ConfigVelocityMeasurementWindow(v_m_window, timeoutMs),"ConfigVelocityMeasurementWindow");

			ts.setVelocityMeasurementPeriod(internal_v_m_period);
			ts.setVelocityMeasurementWindow(v_m_window);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" velocity measurement period / window");
		}

		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			//ROS_WARN("pos");
			safeTalonCall(talon->SetSelectedSensorPosition(sensor_position / radians_scale, pidIdx, timeoutMs),
					"SetSelectedSensorPosition");
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" selected sensor position");

		}

		hardware_interface::LimitSwitchSource internal_local_forward_source;
		hardware_interface::LimitSwitchNormal internal_local_forward_normal;
		hardware_interface::LimitSwitchSource internal_local_reverse_source;
		hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_forward_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_forward_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_reverse_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_reverse_normal;
		if (tc.limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
										  internal_local_reverse_source, internal_local_reverse_normal) &&
				convertLimitSwitchSource(internal_local_forward_source, talon_local_forward_source) &&
				convertLimitSwitchNormal(internal_local_forward_normal, talon_local_forward_normal) &&
				convertLimitSwitchSource(internal_local_reverse_source, talon_local_reverse_source) &&
				convertLimitSwitchNormal(internal_local_reverse_normal, talon_local_reverse_normal) )
		{
			//ROS_WARN("limit_switch");
			safeTalonCall(talon->ConfigForwardLimitSwitchSource(talon_local_forward_source, talon_local_forward_normal, timeoutMs),"ConfigForwardLimitSwitchSource");
			safeTalonCall(talon->ConfigReverseLimitSwitchSource(talon_local_reverse_source, talon_local_reverse_normal, timeoutMs),"ConfigReverseLimitSwitchSource");
			ts.setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
			ts.setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" limit switches");
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
			//ROS_WARN("soft limit");
			double softlimit_forward_threshold_NU = softlimit_forward_threshold / radians_scale; //native units
			double softlimit_reverse_threshold_NU = softlimit_reverse_threshold / radians_scale;
			talon->OverrideSoftLimitsEnable(softlimit_override_enable);
			safeTalonCall(talon->GetLastError(), "OverrideSoftLimitsEnable");
			safeTalonCall(talon->ConfigForwardSoftLimitThreshold(softlimit_forward_threshold_NU, timeoutMs),"ConfigForwardSoftLimitThreshold");
			safeTalonCall(talon->ConfigForwardSoftLimitEnable(softlimit_forward_enable, timeoutMs),"ConfigForwardSoftLimitEnable");
			safeTalonCall(talon->ConfigReverseSoftLimitThreshold(softlimit_reverse_threshold_NU, timeoutMs),"ConfigReverseSoftLimitThreshold");
			safeTalonCall(talon->ConfigReverseSoftLimitEnable(softlimit_reverse_enable, timeoutMs),"ConfigReverseSoftLimitEnable");

			ts.setOverrideSoftLimitsEnable(softlimit_override_enable);
			ts.setForwardSoftLimitThreshold(softlimit_forward_threshold);
			ts.setForwardSoftLimitEnable(softlimit_forward_enable);
			ts.setReverseSoftLimitThreshold(softlimit_reverse_threshold);
			ts.setReverseSoftLimitEnable(softlimit_reverse_enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" soft limits " <<
					std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
					std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
					std::endl << "\toverride_enable=" << softlimit_override_enable);
		}

		int peak_amps;
		int peak_msec;
		int continuous_amps;
		bool enable;
		if (tc.currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
		{
			//ROS_WARN("cur limit");
			safeTalonCall(talon->ConfigPeakCurrentLimit(peak_amps, timeoutMs),"ConfigPeakCurrentLimit");
			safeTalonCall(talon->ConfigPeakCurrentDuration(peak_msec, timeoutMs),"ConfigPeakCurrentDuration");
			safeTalonCall(talon->ConfigContinuousCurrentLimit(continuous_amps, timeoutMs),"ConfigContinuousCurrentLimit");
			talon->EnableCurrentLimit(enable);
			safeTalonCall(talon->GetLastError(), "EnableCurrentLimit");

			ts.setPeakCurrentLimit(peak_amps);
			ts.setPeakCurrentDuration(peak_msec);
			ts.setContinuousCurrentLimit(continuous_amps);
			ts.setCurrentLimitEnable(enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" peak current");
		}

		if (motion_profile_mode)
		{
			double motion_cruise_velocity;
			double motion_acceleration;
			if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration))
			{
				//ROS_WARN("magic changed");
				//converted from rad/sec to native units
				safeTalonCall(talon->ConfigMotionCruiseVelocity((motion_cruise_velocity / radians_per_second_scale), timeoutMs),"ConfigMotionCruiseVelocity(");
				safeTalonCall(talon->ConfigMotionAcceleration((motion_acceleration / radians_per_second_scale), timeoutMs),"ConfigMotionAcceleration(");

				ts.setMotionCruiseVelocity(motion_cruise_velocity);
				ts.setMotionAcceleration(motion_acceleration);

				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" cruise velocity / acceleration");
			}

#if 0 // DISABLE FOR NOW UNTIL WE CAN FIND A SAFE DEFAULT
			// Do this before rest of motion profile stuff
			// so it takes effect before starting a buffer?
			int motion_control_frame_period;
			if (tc.motionControlFramePeriodChanged(motion_control_frame_period))
			{
				//ROS_WARN("profile frame period");
				safeTalonCall(talon->ChangeMotionControlFramePeriod(motion_control_frame_period),"ChangeMotionControlFramePeriod");
				ts.setMotionControlFramePeriod(motion_control_frame_period);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion control frame period");
			}
#endif

			int motion_profile_trajectory_period;
			if (tc.motionProfileTrajectoryPeriodChanged(motion_profile_trajectory_period))
			{
				//ROS_WARN("profile frame period");
				safeTalonCall(talon->ConfigMotionProfileTrajectoryPeriod(motion_profile_trajectory_period, timeoutMs),"ConfigMotionProfileTrajectoryPeriod");
				ts.setMotionProfileTrajectoryPeriod(motion_profile_trajectory_period);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectory period");
			}

			if (tc.clearMotionProfileTrajectoriesChanged())
			{
				//ROS_WARN("clear points");
				safeTalonCall(talon->ClearMotionProfileTrajectories(), "ClearMotionProfileTrajectories");
				(*can_talons_mp_written_)[joint_id].store(false, std::memory_order_relaxed);
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");
			}

			if (tc.clearMotionProfileHasUnderrunChanged())
			{
				//ROS_WARN("clear underrun");
				safeTalonCall(talon->ClearMotionProfileHasUnderrun(timeoutMs),"ClearMotionProfileHasUnderrun");
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile underrun changed");
			}

			// TODO : check that Talon motion buffer is not full
			// before writing, communicate how many have been written
			// - and thus should be cleared - from the talon_command
			// list of requests.
		}

		std::vector<hardware_interface::TrajectoryPoint> trajectory_points;
		if (tc.motionProfileTrajectoriesChanged(trajectory_points))
		{
			//ROS_INFO_STREAM("Pre buffer");
			//ROS_WARN("point_buffer");
			//int i = 0;
			for (auto it = trajectory_points.cbegin(); it != trajectory_points.cend(); ++it)
			{
				ctre::phoenix::motion::TrajectoryPoint pt;
				pt.position = it->position / radians_scale;
				pt.velocity = it->velocity / radians_per_second_scale;
				pt.headingDeg = it->headingRad * 180. / M_PI;
				pt.auxiliaryPos = it->auxiliaryPos; // TODO : unit conversion?
				pt.profileSlotSelect0 = it->profileSlotSelect0;
				pt.profileSlotSelect1 = it->profileSlotSelect1;
				pt.isLastPoint = it->isLastPoint;
				pt.zeroPos = it->zeroPos;
				pt.timeDur = static_cast<ctre::phoenix::motion::TrajectoryDuration>(it->trajectoryDuration);
				safeTalonCall(talon->PushMotionProfileTrajectory(pt),"PushMotionProfileTrajectory");
				//ROS_INFO_STREAM("id: " << joint_id << " pos: " << pt.position << " i: " << i++);
			}
			//ROS_INFO_STREAM("Post buffer");
			// Copy the 1st profile trajectory point from
			// the top level buffer to the talon
			// Subsequent points will be copied by
			// the process_motion_profile_buffer_thread code
			//talon->ProcessMotionProfileBuffer();
			(*can_talons_mp_written_)[joint_id].store(true, std::memory_order_relaxed);

			ROS_INFO_STREAM("Added joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");
		}


		// Set new motor setpoint if either the mode or
		// the setpoint has been changed
		double command;
		hardware_interface::TalonMode in_mode;
		ctre::phoenix::motorcontrol::ControlMode out_mode;

		const bool b1 = tc.newMode(in_mode);
		const bool b2 = tc.commandChanged(command);

		hardware_interface::DemandType demand1_type_internal;
		double demand1_value;
		const bool b3 = tc.demand1Changed(demand1_type_internal, demand1_value);

		if (b1 || b2 || b3)
		{
			if ((b1 || b2) && convertControlMode(in_mode, out_mode))
			{
				ts.setTalonMode(in_mode);
				ts.setSetpoint(command);

				ts.setNeutralOutput(false); // maybe make this a part of setSetpoint?

				switch (out_mode)
				{
					case ctre::phoenix::motorcontrol::ControlMode::Velocity:
						command /= radians_per_second_scale;
						break;
					case ctre::phoenix::motorcontrol::ControlMode::Position:
						command /= radians_scale;
						break;
					case ctre::phoenix::motorcontrol::ControlMode::MotionMagic:
						command /= radians_scale;
						break;
				}
			
				(*can_talons_mp_running_)[joint_id].store(out_mode == ctre::phoenix::motorcontrol::ControlMode::MotionProfile && command == 1, std::memory_order_relaxed);
			}

			ts.setDemand1Type(demand1_type_internal);
			ts.setDemand1Value(demand1_value);

			//ROS_INFO_STREAM c("in mode: " << in_mode);
			if (b3 &&
				(demand1_type_internal > hardware_interface::DemandType::DemandType_Neutral) &&
			    (demand1_type_internal < hardware_interface::DemandType::DemandType_Last) )
			{
				ctre::phoenix::motorcontrol::DemandType demand1_type_phoenix;
				switch (demand1_type_internal)
				{
					case hardware_interface::DemandType::DemandType_AuxPID:
						demand1_type_phoenix = ctre::phoenix::motorcontrol::DemandType::DemandType_AuxPID;
						break;
					case hardware_interface::DemandType::DemandType_ArbitraryFeedForward:
						demand1_type_phoenix = ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward;
						break;
				}

				talon->Set(out_mode, command, demand1_type_phoenix, demand1_value);
			}
			else
				talon->Set(out_mode, command);

			//ROS_WARN_STREAM("set at: " << ts.getCANID() << " new mode: " << b1 << " command_changed: " << b2 << " cmd: " << command);
		}

		if (tc.clearStickyFaultsChanged())
		{
			//ROS_WARN("sticky");
			safeTalonCall(talon->ClearStickyFaults(timeoutMs), "ClearStickyFaults");
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" sticky_faults");
		}
	}
		

	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		nidec_brushlesses_[i]->Set(brushless_command_[i]);
	}

	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		const bool converted_command = (digital_output_command_[i] > 0) ^ digital_output_inverts_[i];
		if (converted_command != digital_output_state_[i])
		{
			digital_outputs_[i]->Set(converted_command);
			digital_output_state_[i] = converted_command;
		}
	}

	for (size_t i = 0; i < num_pwm_; i++)
	{
		const int inverter = (pwm_inverts_[i]) ? -1 : 1;
		PWMs_[i]->SetSpeed(pwm_command_[i]*inverter);
	}
	
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		const bool setpoint = solenoid_command_[i] > 0;
		if (solenoid_state_[i] != setpoint)
		{
			solenoids_[i]->Set(setpoint);
			solenoid_state_[i] = setpoint;
		}
	}

	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		DoubleSolenoid::Value setpoint = DoubleSolenoid::Value::kOff;
		if (double_solenoid_command_[i] >= 1.0)
			setpoint = DoubleSolenoid::Value::kForward;
		else if (double_solenoid_command_[i] <= -1.0)
			setpoint = DoubleSolenoid::Value::kReverse;

		// Not sure if it makes sense to store command values
		// in state or wpilib enum values
		if (double_solenoid_state_[i] != double_solenoid_command_[i])
		{
			double_solenoids_[i]->Set(setpoint);
			double_solenoid_state_[i] = double_solenoid_command_[i];
		}
	}

	for (size_t i = 0; i < num_rumble_; i++)
	{
		if (rumble_state_[i] != rumble_command_[i])
		{
			const unsigned int rumbles = *((unsigned int*)(&rumble_command_[i]));
			const unsigned int left_rumble  = (rumbles >> 16) & 0xFFFF;
			const unsigned int right_rumble = (rumbles      ) & 0xFFFF;
			HAL_SetJoystickOutputs(rumble_ports_[i], 0, left_rumble, right_rumble);
			rumble_state_[i] = rumble_command_[i];
		}
	}

	for (size_t i = 0; i< num_compressors_; i++)
	{
		if (last_compressor_command_[i] != compressor_command_[i])
		{
			const bool setpoint = compressor_command_[i] > 0;
			compressors_[i]->SetClosedLoopControl(setpoint);
			last_compressor_command_[i] = compressor_command_[i];
		}
	}
	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		// Use dummy joints to communicate info between
		// various controllers and driver station smartdash vars
		if (dummy_joint_names_[i] == "cube_state")
		{
			dummy_joint_position_[i] = dummy_joint_command_[i];
			cube_state_.store(dummy_joint_position_[i] != 0, std::memory_order_relaxed);
		}
		else if (dummy_joint_names_[i] == "stop_arm")
			dummy_joint_position_[i] = stop_arm_.load(std::memory_order_relaxed) ? 1 : 0;
		else if (dummy_joint_names_[i] == "override_arm_limits")
			dummy_joint_position_[i] = override_arm_limits_.load(std::memory_order_relaxed) ? 1 : 0;
		else if (dummy_joint_names_[i] == "disable_compressor")
			dummy_joint_position_[i] = disable_compressor_.load(std::memory_order_relaxed) ? 1 : 0;
		else
		{
			dummy_joint_effort_[i] = 0;
			//if (dummy_joint_names_[i].substr(2, std::string::npos) == "_angle")
			{
				// position mode
				dummy_joint_velocity_[i] = (dummy_joint_command_[i] - dummy_joint_position_[i]) / elapsed_time.toSec();
				dummy_joint_position_[i] = dummy_joint_command_[i];
			}
#if 0
			else if (dummy_joint_names_[i].substr(2, std::string::npos) == "_drive")
			{
				// velocity mode
				dummy_joint_position_[i] += dummy_joint_command_[i] * elapsed_time.toSec();
				dummy_joint_velocity_[i] = dummy_joint_command_[i];
			}
#endif
		}
	}
}

// Convert from internal version of hardware mode ID
// to one to write to actual Talon hardware
// Return true if conversion is OK, false if
// an unknown mode is hit.
bool FRCRobotHWInterface::convertControlMode(
	const hardware_interface::TalonMode input_mode,
	ctre::phoenix::motorcontrol::ControlMode &output_mode)
{
	switch (input_mode)
	{
	case hardware_interface::TalonMode_PercentOutput:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
		break;
	case hardware_interface::TalonMode_Position:      // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Position;
		break;
	case hardware_interface::TalonMode_Velocity:      // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
		break;
	case hardware_interface::TalonMode_Current:       // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Current;
		break;
	case hardware_interface::TalonMode_Follower:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Follower;
		break;
	case hardware_interface::TalonMode_MotionProfile:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionProfile;
		break;
	case hardware_interface::TalonMode_MotionMagic:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
		break;
	case hardware_interface::TalonMode_Disabled:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		break;
	default:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		ROS_WARN("Unknown mode seen in HW interface");
		return false;
	}
	return true;
}

bool FRCRobotHWInterface::convertNeutralMode(
	const hardware_interface::NeutralMode input_mode,
	ctre::phoenix::motorcontrol::NeutralMode &output_mode)
{
	switch (input_mode)
	{
	case hardware_interface::NeutralMode_EEPROM_Setting:
		output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
		break;
	case hardware_interface::NeutralMode_Coast:
		output_mode = ctre::phoenix::motorcontrol::Coast;
		break;
	case hardware_interface::NeutralMode_Brake:
		output_mode = ctre::phoenix::motorcontrol::Brake;
		break;
	default:
		output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
		ROS_WARN("Unknown neutral mode seen in HW interface");
		return false;
	}

	return true;
}

bool FRCRobotHWInterface::convertFeedbackDevice(
	const hardware_interface::FeedbackDevice input_fd,
	ctre::phoenix::motorcontrol::FeedbackDevice &output_fd)
{
	switch (input_fd)
	{
	case hardware_interface::FeedbackDevice_QuadEncoder:
		output_fd = ctre::phoenix::motorcontrol::QuadEncoder;
		break;
	case hardware_interface::FeedbackDevice_Analog:
		output_fd = ctre::phoenix::motorcontrol::Analog;
		break;
	case hardware_interface::FeedbackDevice_Tachometer:
		output_fd = ctre::phoenix::motorcontrol::Tachometer;
		break;
	case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
		output_fd = ctre::phoenix::motorcontrol::PulseWidthEncodedPosition;
		break;
	case hardware_interface::FeedbackDevice_SensorSum:
		output_fd = ctre::phoenix::motorcontrol::SensorSum;
		break;
	case hardware_interface::FeedbackDevice_SensorDifference:
		output_fd = ctre::phoenix::motorcontrol::SensorDifference;
		break;
	case hardware_interface::FeedbackDevice_RemoteSensor0:
		output_fd = ctre::phoenix::motorcontrol::RemoteSensor0;
		break;
	case hardware_interface::FeedbackDevice_RemoteSensor1:
		output_fd = ctre::phoenix::motorcontrol::RemoteSensor1;
		break;
	case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
		output_fd = ctre::phoenix::motorcontrol::SoftwareEmulatedSensor;
		break;
	default:
		ROS_WARN("Unknown feedback device seen in HW interface");
		return false;
	}
	return true;
}

bool FRCRobotHWInterface::convertLimitSwitchSource(
	const hardware_interface::LimitSwitchSource input_ls,
	ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls)
{
	switch (input_ls)
	{
	case hardware_interface::LimitSwitchSource_FeedbackConnector:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
		break;
	case hardware_interface::LimitSwitchSource_RemoteTalonSRX:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
		break;
	case hardware_interface::LimitSwitchSource_RemoteCANifier:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteCANifier;
		break;
	case hardware_interface::LimitSwitchSource_Deactivated:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
		break;
	default:
		ROS_WARN("Unknown limit switch source seen in HW interface");
		return false;
	}
	return true;
}

bool FRCRobotHWInterface::convertLimitSwitchNormal(
	const hardware_interface::LimitSwitchNormal input_ls,
	ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls)
{
	switch (input_ls)
	{
	case hardware_interface::LimitSwitchNormal_NormallyOpen:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
		break;
	case hardware_interface::LimitSwitchNormal_NormallyClosed:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
		break;
	case hardware_interface::LimitSwitchNormal_Disabled:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
		break;
	default:
		ROS_WARN("Unknown limit switch normal seen in HW interface");
		return false;
	}
	return true;

}

bool FRCRobotHWInterface::convertVelocityMeasurementPeriod(const hardware_interface::VelocityMeasurementPeriod input_v_m_p, ctre::phoenix::motorcontrol::VelocityMeasPeriod &output_v_m_period)
{
	switch(input_v_m_p)
	{
		case hardware_interface::VelocityMeasurementPeriod::Period_1Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_1Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_2Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_2Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_5Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_10Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_10Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_20Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_20Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_25Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_25Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_50Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_50Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_100Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_100Ms;
			break;
		default:
			ROS_WARN("Unknown velocity measurement period seen in HW interface");
			return false;
	}
	return true;
}

} // namespace 
