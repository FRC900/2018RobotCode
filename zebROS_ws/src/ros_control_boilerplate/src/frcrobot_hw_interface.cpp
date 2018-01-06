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

#include <iostream>
#include <thread>

#include <ros_control_boilerplate/frcrobot_hw_interface.h>
#include <ros_control_boilerplate/JoystickState.h>
#include "HAL/DriverStation.h"
#include "HAL/HAL.h"
#include "Joystick.h"
//#include "GenericHID.h"
#include "math.h"

namespace frcrobot_control
{
// TODO : figure out what these mean
const int pidIdx = 0;
const int timeoutMs = 0;

//TODO Make nativeU configurable
int nativeU = 4096;   //native units of ctre magnetic encoders
//RG: More than just making nativeU configurable, we should consider a much more automated system
//i.e. set conversion factor based on specified feedback sensor
//note that you can add a conversion factors that will automatically be applied for speed and position


FRCRobotHWInterface::FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model),
	  run_hal_thread_(true)
{
}

FRCRobotHWInterface::~FRCRobotHWInterface()
{
	run_hal_thread_ = false;
	hal_thread_.join();
}

void FRCRobotHWInterface::hal_keepalive_thread(void)
{
	// Just throw a basic IterativeRobot in here instead?
	run_hal_thread_ = true;
	Joystick joystick(0);
	realtime_tools::RealtimePublisher<ros_control_boilerplate::JoystickState> realtime_pub(nh_, "joystick_states", 4);
	while (run_hal_thread_)
	{
		robot_.OneIteration();
		// Things to keep track of
		//    Alliance Station Id
		//    Robot / match mode (auto, teleop, test, disabled)
		//    Match time
		match_time_state_ = DriverStation::GetInstance().GetMatchTime();
		//    Joystick state
		//    This is for testing. Need to expand this
		//    to include buttons, the ability to set
		//    rumble state via an external joystick
		//    controller, etc.  Need to set via config
		//    files.

		//TODO: match gets with correct labels
		/*
		joystick_state_[0].leftStickX  = joystick.GetRawAxis(0);
		joystick_state_[0].leftStickY  = joystick.GetRawAxis(1);
		joystick_state_[0].rightStickX = joystick.GetRawAxis(4);
		joystick_state_[0].rightStickY = joystick.GetRawAxis(5);
		joystick_state_[0].leftTrigger = joystick.GetRawAxis(2);
		joystick_state_[0].rightTrigger= joystick.GetRawAxis(3);

		joystick_state_[0].buttonA.button   	 = joystick.GetRawButton(1);
		joystick_state_[0].buttonA.press    	 = joystick.GetRawButtonPressed(1);
		joystick_state_[0].buttonA.release  	 = joystick.GetRawButtonReleased(1);

		joystick_state_[0].buttonB.button   	 = joystick.GetRawButton(2);
		joystick_state_[0].buttonB.press    	 = joystick.GetRawButtonPressed(2);
		joystick_state_[0].buttonB.release  	 = joystick.GetRawButtonReleased(2);

		joystick_state_[0].buttonX.button   	 = joystick.GetRawButton(3);
		joystick_state_[0].buttonX.press    	 = joystick.GetRawButtonPressed(3);
		joystick_state_[0].buttonX.release  	 = joystick.GetRawButtonReleased(3);

		joystick_state_[0].buttonY.button   	 = joystick.GetRawButton(4);
		joystick_state_[0].buttonY.press    	 = joystick.GetRawButtonPressed(4);
		joystick_state_[0].buttonY.release  	 = joystick.GetRawButtonReleased(4);

		joystick_state_[0].bumperLeft.button   	 = joystick.GetRawButton(5);
		joystick_state_[0].bumperLeft.press    	 = joystick.GetRawButtonPressed(5);
		joystick_state_[0].bumperLeft.release  	 = joystick.GetRawButtonReleased(5);

		joystick_state_[0].bumperRight.button    = joystick.GetRawButton(6);
		joystick_state_[0].bumperRight.press     = joystick.GetRawButtonPressed(6);
		joystick_state_[0].bumperRight.release   = joystick.GetRawButtonReleased(6);

		joystick_state_[0].buttonBack.button   	 = joystick.GetRawButton(7);
		joystick_state_[0].buttonBack.press    	 = joystick.GetRawButtonPressed(7);
		joystick_state_[0].buttonBack.release  	 = joystick.GetRawButtonReleased(7);

		joystick_state_[0].buttonStart.button    = joystick.GetRawButton(8);
		joystick_state_[0].buttonStart.press   	 = joystick.GetRawButtonPressed(8);
		joystick_state_[0].buttonStart.release 	 = joystick.GetRawButtonReleased(8);

		joystick_state_[0].stickLeft.button	     = joystick.GetRawButton(9);
		joystick_state_[0].stickLeft.press   	 = joystick.GetRawButtonPressed(9);
		joystick_state_[0].stickLeft.release 	 = joystick.GetRawButtonReleased(9);

		joystick_state_[0].stickRight.button     = joystick.GetRawButton(10);
		joystick_state_[0].stickRight.press   	 = joystick.GetRawButtonPressed(10);
		joystick_state_[0].stickRight.release 	 = joystick.GetRawButtonReleased(10);

		*/

		if (realtime_pub.trylock())
		{
			realtime_pub.msg_.header.stamp = ros::Time::now();

			realtime_pub.msg_.leftStickX = joystick.GetRawAxis(0);
			realtime_pub.msg_.leftStickY = joystick.GetRawAxis(1);
			realtime_pub.msg_.rightStickX = joystick.GetRawAxis(4);
			realtime_pub.msg_.rightStickY = joystick.GetRawAxis(5);

			realtime_pub.msg_.leftTrigger = joystick.GetRawAxis(2);
			realtime_pub.msg_.rightTrigger = joystick.GetRawAxis(3);

			realtime_pub.msg_.buttonAButton = joystick.GetRawButton(1);
			realtime_pub.msg_.buttonAPress = joystick.GetRawButtonPressed(1);
			realtime_pub.msg_.buttonARelease = joystick.GetRawButtonReleased(1);

			realtime_pub.msg_.buttonBButton = joystick.GetRawButton(2);
			realtime_pub.msg_.buttonBPress = joystick.GetRawButtonPressed(2);
			realtime_pub.msg_.buttonBRelease = joystick.GetRawButtonReleased(2);

			realtime_pub.msg_.buttonXButton = joystick.GetRawButton(3);
			realtime_pub.msg_.buttonXPress = joystick.GetRawButtonPressed(3);
			realtime_pub.msg_.buttonXRelease = joystick.GetRawButtonReleased(3);

			realtime_pub.msg_.buttonYButton = joystick.GetRawButton(4);
			realtime_pub.msg_.buttonYPress = joystick.GetRawButtonPressed(4);
			realtime_pub.msg_.buttonYRelease = joystick.GetRawButtonReleased(4);

			realtime_pub.msg_.bumperLeftButton = joystick.GetRawButton(5);
			realtime_pub.msg_.bumperLeftPress = joystick.GetRawButtonPressed(5);
			realtime_pub.msg_.bumperLeftRelease = joystick.GetRawButtonReleased(5);

			realtime_pub.msg_.bumperRightButton = joystick.GetRawButton(6);
			realtime_pub.msg_.bumperRightPress = joystick.GetRawButtonPressed(6);
			realtime_pub.msg_.bumperRightRelease = joystick.GetRawButtonReleased(6);

			realtime_pub.msg_.buttonBackButton = joystick.GetRawButton(7);
			realtime_pub.msg_.buttonBackPress = joystick.GetRawButtonPressed(7);
			realtime_pub.msg_.buttonBackRelease = joystick.GetRawButtonReleased(7);

			realtime_pub.msg_.buttonStartButton = joystick.GetRawButton(8);
			realtime_pub.msg_.buttonStartPress = joystick.GetRawButtonPressed(8);
			realtime_pub.msg_.buttonStartRelease = joystick.GetRawButtonReleased(8);

			realtime_pub.msg_.stickLeftButton = joystick.GetRawButton(9);
			realtime_pub.msg_.stickLeftPress = joystick.GetRawButtonPressed(9);
			realtime_pub.msg_.stickLeftRelease = joystick.GetRawButtonReleased(9);

			realtime_pub.msg_.stickRightButton = joystick.GetRawButton(10);
			realtime_pub.msg_.stickRightPress = joystick.GetRawButtonPressed(10);
			realtime_pub.msg_.stickRightRelease = joystick.GetRawButtonReleased(10);

			realtime_pub.unlockAndPublish();
		}
		//TODO: Add direction buttons?
	}
}

void FRCRobotHWInterface::init(void)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	FRCRobotInterface::init();

	// Make sure to initialize WPIlib code before creating
	// a CAN Talon object to avoid NIFPGA: Resource not initialized
	// errors? See https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
	robot_.StartCompetition();
	hal_thread_ = std::thread(&FRCRobotHWInterface::hal_keepalive_thread, this);

	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		can_talons_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(can_talon_srx_can_ids_[i] /*, CAN update rate*/ ));

		// Need config information for each talon
		// Should probably be part of YAML params for controller
		// set close loop ramp rate - same as above
		// set voltage compensation rate
		// set soft limits - forward/reverse limits and enables - yes
		// set limit switch config - enable, NO/NC  - probably yes

		can_talons_[i]->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0); // Make sure motor is stopped
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		nidec_brushlesses_.push_back(std::make_shared<frc::NidecBrushless>(nidec_brushless_pwm_channels_[i], nidec_brushless_dio_channels_[i]));
		nidec_brushlesses_[i]->SetInverted(nidec_brushless_inverts_[i]);
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		digital_inputs_.push_back(std::make_shared<frc::DigitalInput>(digital_input_dio_channels_[i]));
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		digital_outputs_.push_back(std::make_shared<frc::DigitalOutput>(digital_output_dio_channels_[i]));
	}
	for (size_t i = 0; i < num_pwm_; i++)
	{
		PWMs_.push_back(std::make_shared<frc::SafePWM>(pwm_pwm_channels_[i]));
		PWMs_[i]->SetSafetyEnabled(false);
	}

	ROS_INFO_NAMED("frcrobot_hw_interface", "FRCRobotHWInterface Ready.");
}

void FRCRobotHWInterface::read(ros::Duration &/*elapsed_time*/)
{
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		// read position and velocity from can_talons_[joint_id]
		// convert to whatever units make sense
		//
		// TODO : convert to units which make sense
		// for rest of code? Add a method which takes current
		// mode, encoder choice and maybe a user-configurable ticks/rotation
		// setting and converts from native units to radians (for position)
		// and radians/src (for velocity)
		talon_state_[joint_id].setPosition(can_talons_[joint_id]->GetSelectedSensorPosition(pidIdx) / 4096. * 2 * M_PI);
		talon_state_[joint_id].setSpeed(can_talons_[joint_id]->GetSelectedSensorVelocity(pidIdx) / 4096. * 2 * M_PI / .1);

		double bus_voltage = can_talons_[joint_id]->GetBusVoltage();
		talon_state_[joint_id].setBusVoltage(bus_voltage);

		double motor_output_percent = can_talons_[joint_id]->GetMotorOutputPercent();
		talon_state_[joint_id].setMotorOutputPercent(motor_output_percent);

		double output_voltage = can_talons_[joint_id]->GetMotorOutputVoltage();
		talon_state_[joint_id].setOutputVoltage(output_voltage);
		double output_current = can_talons_[joint_id]->GetOutputCurrent();
		talon_state_[joint_id].setOutputCurrent(output_current);

		double temperature = can_talons_[joint_id]->GetTemperature();
		talon_state_[joint_id].setTemperature(temperature);

		// Scale this from native units
		double scale = 1.;

		if (talon_state_[joint_id].getTalonMode() == hardware_interface::TalonMode_Position)
			scale = 4096.*2 * M_PI;
		else if (talon_state_[joint_id].getTalonMode() == hardware_interface::TalonMode_Velocity)
			scale = 4096.*2 * M_PI / .1;
		double closed_loop_error = can_talons_[joint_id]->GetClosedLoopError(pidIdx) / scale;
		talon_state_[joint_id].setClosedLoopError(closed_loop_error);
		ROS_INFO_STREAM_THROTTLE(1, std::endl << "ClosedLoopError:" << closed_loop_error);

		double integral_accumulator = can_talons_[joint_id]->GetIntegralAccumulator(pidIdx) / scale;
		talon_state_[joint_id].setIntegralAccumulator(integral_accumulator);

		double error_derivative = can_talons_[joint_id]->GetErrorDerivative(pidIdx) / scale;
		talon_state_[joint_id].setErrorDerivative(error_derivative);

		// TODO : scale me?
		double closed_loop_target = can_talons_[joint_id]->GetClosedLoopTarget(pidIdx);
		talon_state_[joint_id].setClosedLoopTarget(closed_loop_target);

		double active_trajectory_position = can_talons_[joint_id]->GetActiveTrajectoryPosition();
		talon_state_[joint_id].setActiveTrajectoryPosition(active_trajectory_position);
		double active_trajectory_velocity = can_talons_[joint_id]->GetActiveTrajectoryVelocity();
		talon_state_[joint_id].setActiveTrajectoryVelocity(active_trajectory_velocity);
		double active_trajectory_heading = can_talons_[joint_id]->GetActiveTrajectoryHeading();
		talon_state_[joint_id].setActiveTrajectoryHeading(active_trajectory_heading);


		// TODO : only do this if mode is motion profile or
		// motion magic?
		talon_state_[joint_id].setMotionProfileTopLevelBufferCount(can_talons_[joint_id]->GetMotionProfileTopLevelBufferCount());
		talon_state_[joint_id].setMotionProfileTopLevelBufferFull(can_talons_[joint_id]->IsMotionProfileTopLevelBufferFull());
		ctre::phoenix::motion::MotionProfileStatus talon_status;
		can_talons_[joint_id]->GetMotionProfileStatus(talon_status);

		hardware_interface::MotionProfileStatus internal_status;
		internal_status.topBufferRem = talon_status.topBufferRem;
		internal_status.topBufferCnt = talon_status.topBufferCnt;
		internal_status.btmBufferCnt = talon_status.btmBufferCnt;
		internal_status.hasUnderrun = talon_status.hasUnderrun;
		internal_status.isUnderrun = talon_status.isUnderrun;
		internal_status.activePointValid = talon_status.activePointValid;
		internal_status.isLast = talon_status.isLast;
		internal_status.profileSlotSelect = talon_status.profileSlotSelect;
		internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);

		talon_state_[joint_id].setMotionProfileStatus(internal_status);


		// TODO :: Fix me
		//talon_state_[joint_id].setFwdLimitSwitch(can_talons_[joint_id]->IsFwdLimitSwitchClosed());
		//talon_state_[joint_id].setRevLimitSwitch(can_talons_[joint_id]->IsRevLimitSwitchClosed());
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		// TODO : Figure out which of these the setpoint
		// actually is...
		brushless_pos_[i] =
			brushless_vel_[i] =
				brushless_eff_[i] = nidec_brushlesses_[i]->Get();
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		digital_input_state_[i] = (digital_inputs_[i]->Get()^digital_input_inverts_[i]) ? 1 : 0;
		//State should really be a bool
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		digital_output_state_[i] = (digital_outputs_[i]->Get()^digital_output_inverts_[i]) ? 1 : 0;
		//State should really be a bool
		//This isn't strictly neccesary, it just reads what the DIO is currently set to
	}
	/*
	for (size_t i = 0; i < num_pwm_; i++)
	{
		//Nothing to read
	//// TODO : Add a read of state just so we can monitor what's going on in the code
	}
	*/

}

//get rid of magic numbers
double FRCRobotHWInterface::getRadiansConversionFactor(hardware_interface::FeedbackDevice encoder_feedback, int joint_id) //convert to radians //how to include talon_mode?
{
	switch (encoder_feedback)
	{
	case hardware_interface::FeedbackDevice_QuadEncoder:
	case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
		return 2 * M_PI / 4056; //4056 = 4* encoder cycles per revolution
	case hardware_interface::FeedbackDevice_Analog:
		return 2 * M_PI / 1024;
	case hardware_interface::FeedbackDevice_Tachometer:
	case hardware_interface::FeedbackDevice_SensorSum:
	case hardware_interface::FeedbackDevice_SensorDifference:
	case hardware_interface::FeedbackDevice_RemoteSensor0:
	case hardware_interface::FeedbackDevice_RemoteSensor1:
	case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
		ROS_WARN_STREAM("Unable to convert units.");
		return 1;
	default:
		ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
		return 1;
	}
}

double FRCRobotHWInterface::getRadiansPerSecConversionFactor(hardware_interface::FeedbackDevice encoder_feedback, int joint_id) //convert to radians/sec from native units/.1sec
{
	switch (encoder_feedback)
	{
	case hardware_interface::FeedbackDevice_QuadEncoder:
	case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
		return 2 * M_PI / 4056 / .1; //4056 = 4* encoder cycles per revolution
	case hardware_interface::FeedbackDevice_Analog:
		return 2 * M_PI / 1024 / .1;
	case hardware_interface::FeedbackDevice_Tachometer:
	case hardware_interface::FeedbackDevice_SensorSum:
	case hardware_interface::FeedbackDevice_SensorDifference:
	case hardware_interface::FeedbackDevice_RemoteSensor0:
	case hardware_interface::FeedbackDevice_RemoteSensor1:
	case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
		ROS_WARN_STREAM("Unable to convert units.");
		return 1;
	default:
		ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
		return 1;
	}
}


void FRCRobotHWInterface::write(ros::Duration &elapsed_time)
{
	// Safety - should be using Talon HW to control this
	// Maybe do a eStop / enabled check instead?
	//enforceLimits(elapsed_time);

	// ----------------------------------------------------
	// ----------------------------------------------------
	// ----------------------------------------------------
	//
//	ROS_INFO_STREAM_THROTTLE(1, std::endl << std::string(__FILE__) << ":" << __LINE__ <<
//			                    std::endl << printCommandHelper());

	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		int slot;
		if (talon_command_[joint_id].slotChanged(slot))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF slot to " << slot << std::endl);

			can_talons_[joint_id]->SelectProfileSlot(slot, timeoutMs);
			talon_state_[joint_id].setSlot(slot);
		}

		hardware_interface::FeedbackDevice internal_feedback_device;
		ctre::phoenix::motorcontrol::FeedbackDevice talon_feedback_device;
		if (talon_command_[joint_id].encoderFeedbackChanged(internal_feedback_device) &&
				convertFeedbackDevice(internal_feedback_device, talon_feedback_device))
		{
			can_talons_[joint_id]->ConfigSelectedFeedbackSensor(talon_feedback_device, pidIdx, timeoutMs);
			talon_state_[joint_id].setEncoderFeedback(internal_feedback_device);
		}

		double p;
		double i;
		double d;
		double f;
		int    iz;
		int    allowable_closed_loop_error;
		double max_integral_accumulator;
		for (int j = 0; j < 2; j++)
		{
			if (talon_command_[joint_id].pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, j))
			{
				can_talons_[joint_id]->Config_kP(j, p, timeoutMs);
				can_talons_[joint_id]->Config_kI(j, i, timeoutMs);
				can_talons_[joint_id]->Config_kD(j, d, timeoutMs);
				can_talons_[joint_id]->Config_kF(j, f, timeoutMs);
				can_talons_[joint_id]->Config_IntegralZone(j, iz, timeoutMs);
				can_talons_[joint_id]->ConfigAllowableClosedloopError(j, allowable_closed_loop_error, timeoutMs);
				can_talons_[joint_id]->ConfigMaxIntegralAccumulator(j, max_integral_accumulator, timeoutMs);

				talon_state_[joint_id].setPidfP(p, j);
				talon_state_[joint_id].setPidfI(i, j);
				talon_state_[joint_id].setPidfD(d, j);
				talon_state_[joint_id].setPidfF(f, j);
				talon_state_[joint_id].setPidfIzone(iz, j);
				talon_state_[joint_id].setAllowableClosedLoopError(allowable_closed_loop_error, j);
				talon_state_[joint_id].setMaxIntegralAccumulator(max_integral_accumulator, j);
			}
		}

		bool invert;
		bool sensor_phase;
		if (talon_command_[joint_id].invertChanged(invert, sensor_phase))
		{
			can_talons_[joint_id]->SetInverted(invert);
			can_talons_[joint_id]->SetSensorPhase(sensor_phase);
			talon_state_[joint_id].setInvert(invert);
			talon_state_[joint_id].setSensorPhase(sensor_phase);
		}

		hardware_interface::NeutralMode neutral_mode;
		ctre::phoenix::motorcontrol::NeutralMode ctre_neutral_mode;

		if (talon_command_[joint_id].neutralModeChanged(neutral_mode) &&
				convertNeutralMode(neutral_mode, ctre_neutral_mode))
		{
			can_talons_[joint_id]->SetNeutralMode(ctre_neutral_mode);
			talon_state_[joint_id].setNeutralMode(neutral_mode);
		}

		if (talon_command_[joint_id].neutralOutputChanged())
		{
			can_talons_[joint_id]->NeutralOutput();
			talon_state_[joint_id].setNeutralOutput(true);
		}

		double iaccum;
		if (talon_command_[joint_id].integralAccumulatorChanged(iaccum))
		{
			can_talons_[joint_id]->SetIntegralAccumulator(iaccum, pidIdx, timeoutMs);
			// Do not set talon state - this changes
			// dynamically so read it in read() above instead
		}

		double closed_loop_ramp;
		double open_loop_ramp;
		double peak_output_forward;
		double peak_output_reverse;
		double nominal_output_forward;
		double nominal_output_reverse;
		double neutral_deadband;
		if (talon_command_[joint_id].outputShapingChanged(closed_loop_ramp,
				open_loop_ramp,
				peak_output_forward,
				peak_output_reverse,
				nominal_output_forward,
				nominal_output_reverse,
				neutral_deadband))
		{
			can_talons_[joint_id]->ConfigOpenloopRamp(open_loop_ramp, timeoutMs);
			can_talons_[joint_id]->ConfigClosedloopRamp(closed_loop_ramp, timeoutMs);
			can_talons_[joint_id]->ConfigPeakOutputForward(peak_output_forward, timeoutMs);
			can_talons_[joint_id]->ConfigPeakOutputReverse(peak_output_reverse, timeoutMs);
			can_talons_[joint_id]->ConfigNominalOutputForward(nominal_output_forward, timeoutMs);
			can_talons_[joint_id]->ConfigNominalOutputReverse(nominal_output_reverse, timeoutMs);
			can_talons_[joint_id]->ConfigNeutralDeadband(neutral_deadband, timeoutMs);

			talon_state_[joint_id].setOpenloopRamp(open_loop_ramp);
			talon_state_[joint_id].setClosedloopRamp(closed_loop_ramp);
			talon_state_[joint_id].setPeakOutputForward(peak_output_forward);
			talon_state_[joint_id].setPeakOutputReverse(peak_output_reverse);
			talon_state_[joint_id].setNominalOutputForward(nominal_output_forward);
			talon_state_[joint_id].setNominalOutputReverse(nominal_output_reverse);
		}
		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (talon_command_[joint_id].VoltageCompensationChanged(v_c_saturation,
				v_measurement_filter,
				v_c_enable))
		{
			can_talons_[joint_id]->ConfigVoltageCompSaturation(v_c_saturation, timeoutMs);
			can_talons_[joint_id]->ConfigVoltageMeasurementFilter(v_measurement_filter, timeoutMs);
			can_talons_[joint_id]->EnableVoltageCompensation(v_c_enable);

			talon_state_[joint_id].setVoltageCompensationSaturation(v_c_saturation);
			talon_state_[joint_id].setVoltageMeasurementFilter(v_measurement_filter);
			talon_state_[joint_id].setVoltageCompensationEnable(v_c_enable);
		}

		hardware_interface::LimitSwitchSource internal_local_forward_source;
		hardware_interface::LimitSwitchNormal internal_local_forward_normal;
		hardware_interface::LimitSwitchSource internal_local_reverse_source;
		hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_forward_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_forward_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_reverse_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_reverse_normal;
		if (talon_command_[joint_id].limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
				internal_local_reverse_source, internal_local_reverse_normal) &&
				convertLimitSwitchSource(internal_local_forward_source, talon_local_forward_source) &&
				convertLimitSwitchNormal(internal_local_forward_normal, talon_local_forward_normal) &&
				convertLimitSwitchSource(internal_local_reverse_source, talon_local_reverse_source) &&
				convertLimitSwitchNormal(internal_local_reverse_normal, talon_local_reverse_normal) )
		{
			can_talons_[joint_id]->ConfigForwardLimitSwitchSource(talon_local_forward_source, talon_local_forward_normal, timeoutMs);
			can_talons_[joint_id]->ConfigReverseLimitSwitchSource(talon_local_reverse_source, talon_local_reverse_normal, timeoutMs);
			talon_state_[joint_id].setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
			talon_state_[joint_id].setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
		}

		double softlimit_forward_threshold;
		bool softlimit_forward_enable;
		double softlimit_reverse_threshold;
		bool softlimit_reverse_enable;
		bool softlimit_override_enable;
		if (talon_command_[joint_id].SoftLimitChanged(softlimit_forward_threshold,
				softlimit_forward_enable,
				softlimit_reverse_threshold,
				softlimit_reverse_enable,
				softlimit_override_enable))
		{
			//TODO : scale forward and reverse thresholds
			can_talons_[joint_id]->ConfigForwardSoftLimitThreshold(softlimit_forward_threshold, timeoutMs);
			can_talons_[joint_id]->ConfigForwardSoftLimitEnable(softlimit_forward_enable, timeoutMs);
			can_talons_[joint_id]->ConfigReverseSoftLimitThreshold(softlimit_reverse_threshold, timeoutMs);
			can_talons_[joint_id]->ConfigReverseSoftLimitEnable(softlimit_reverse_enable, timeoutMs);
			can_talons_[joint_id]->OverrideSoftLimitsEnable(softlimit_override_enable);
			talon_state_[joint_id].setForwardSoftLimitThreshold(softlimit_forward_threshold);
			talon_state_[joint_id].setForwardSoftLimitEnable(softlimit_forward_enable);
			talon_state_[joint_id].setReverseSoftLimitThreshold(softlimit_reverse_threshold);
			talon_state_[joint_id].setReverseSoftLimitEnable(softlimit_reverse_enable);
			talon_state_[joint_id].setOverrideSoftLimitsEnable(softlimit_override_enable);
		}

		int peak_amps;
		int peak_msec;
		int continuous_amps;
		bool enable;
		if (talon_command_[joint_id].currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
		{
			can_talons_[joint_id]->ConfigPeakCurrentLimit(peak_amps, timeoutMs);
			can_talons_[joint_id]->ConfigPeakCurrentDuration(peak_msec, timeoutMs);
			can_talons_[joint_id]->ConfigContinuousCurrentLimit(continuous_amps, timeoutMs);
			can_talons_[joint_id]->EnableCurrentLimit(enable);

			talon_state_[joint_id].setPeakCurrentLimit(peak_amps);
			talon_state_[joint_id].setPeakCurrentDuration(peak_msec);
			talon_state_[joint_id].setContinuousCurrentLimit(continuous_amps);
			talon_state_[joint_id].setCurrentLimitEnable(enable);
		}

		double motion_cruise_velocity;
		double motion_acceleration;
		if (talon_command_[joint_id].motionCruiseChanged(motion_cruise_velocity, motion_acceleration))
		{
			talon_state_[joint_id].setMotionCruiseVelocity(motion_cruise_velocity);
			talon_state_[joint_id].setMotionAcceleration(motion_acceleration);
			// TODO : covert from rad/sec to native units

			can_talons_[joint_id]->ConfigMotionCruiseVelocity(motion_cruise_velocity, timeoutMs);
			can_talons_[joint_id]->ConfigMotionAcceleration(motion_acceleration, timeoutMs);
		}
		// Do this before rest of motion profile stuff
		// so it takes effect before starting a buffer?
		int motion_control_frame_period;
		if (talon_command_[joint_id].motionControlFramePeriodChanged(motion_control_frame_period))
		{
			can_talons_[joint_id]->ChangeMotionControlFramePeriod(motion_control_frame_period);
			talon_state_[joint_id].setMotionControlFramePeriod(motion_control_frame_period);
		}

		if (talon_command_[joint_id].clearMotionProfileTrajectoriesChanged())
			can_talons_[joint_id]->ClearMotionProfileTrajectories();

		if (talon_command_[joint_id].clearMotionProfileHasUnderrunChanged())
			can_talons_[joint_id]->ClearMotionProfileHasUnderrun(timeoutMs);

		std::vector<hardware_interface::TrajectoryPoint> trajectory_points;

		if (talon_command_[joint_id].motionProfileTrajectoriesChanged(trajectory_points))
		{
			for (auto it = trajectory_points.cbegin(); it != trajectory_points.cend(); ++it)
			{
				ctre::phoenix::motion::TrajectoryPoint pt;
				pt.position = it->position;
				pt.velocity = it->velocity;
				pt.headingDeg = it->headingRad * 180. / M_PI;
				pt.profileSlotSelect = it->profileSlotSelect;
				pt.isLastPoint = it->isLastPoint;
				pt.zeroPos = it->zeroPos;
				can_talons_[joint_id]->PushMotionProfileTrajectory(pt);
			}
		}

		if (talon_command_[joint_id].processMotionProfileBufferChanged())
			can_talons_[joint_id]->ProcessMotionProfileBuffer();

		// Set new motor setpoint if either the mode or
		// the setpoint has been changed
		double command;
		hardware_interface::TalonMode in_mode;
		ctre::phoenix::motorcontrol::ControlMode out_mode;
		if ((talon_command_[joint_id].newMode(in_mode) ||
				talon_command_[joint_id].get(command) ) &&
				convertControlMode(in_mode, out_mode))
		{
			switch (out_mode)
			{
			case ctre::phoenix::motorcontrol::ControlMode::Velocity:
				command = command / 2. / M_PI * nativeU * .1; //assumes input value is velocity per 100ms there is a chance it is supposed to be 10ms
				//RG: I am almost certain that it isn't 10 ms. However, if you configure some of the units
				//using one of the talon functions,  the units are RPM and Rotations
				break;
			case ctre::phoenix::motorcontrol::ControlMode::Position:
				command = command / 2. / M_PI * nativeU;
				break;
			}
			can_talons_[joint_id]->Set(out_mode, command);
			talon_state_[joint_id].setTalonMode(in_mode);
			talon_state_[joint_id].setSetpoint(command);
			talon_state_[joint_id].setNeutralOutput(false); // maybe make this a part of setSetpoint?
		}
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		nidec_brushlesses_[i]->Set(brushless_command_[i]);
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		bool converted_command = (digital_output_command_[i] > 0) ^ digital_output_inverts_[i];
		digital_outputs_[i]->Set(converted_command);
	}
	for (size_t i = 0; i < num_pwm_; i++)
	{
		int inverter  = (pwm_inverts_[i]) ? -1 : 1;
		PWMs_[i]->SetSpeed(pwm_command_[i]*inverter);
	}

}

void FRCRobotHWInterface::enforceLimits(ros::Duration &period)
{
	// ----------------------------------------------------
	// ----------------------------------------------------
	// ----------------------------------------------------
	//
	// CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
	// YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
	// DEPENDING ON YOUR CONTROL METHOD
	//
	// EXAMPLES:
	//
	// Saturation Limits ---------------------------
	//
	// Enforces position and velocity
	pos_jnt_sat_interface_.enforceLimits(period);
	//
	// Enforces velocity and acceleration limits
	// vel_jnt_sat_interface_.enforceLimits(period);
	//
	// Enforces position, velocity, and effort
	// eff_jnt_sat_interface_.enforceLimits(period);

	// Soft limits ---------------------------------
	//
	// pos_jnt_soft_limits_.enforceLimits(period);
	// vel_jnt_soft_limits_.enforceLimits(period);
	// eff_jnt_soft_limits_.enforceLimits(period);
	//
	// ----------------------------------------------------
	// ----------------------------------------------------
	// ----------------------------------------------------
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
	case hardware_interface::TalonMode_TimedPercentOutput:
		//output_mode = ctre::phoenix::motorcontrol::ControlMode::TimedPercentOutput;
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		ROS_WARN("TimedPercentOutput mode seen in HW interface");
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

}  // namespace
