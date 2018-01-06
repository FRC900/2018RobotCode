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

namespace frcrobot_control
{

FRCRobotSimInterface::FRCRobotSimInterface(ros::NodeHandle &nh,
		urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
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
							  "Loading joint " << can_talon_srx_names_[i] <<
							  " as CAN id " << can_talon_srx_can_ids_[i]);

		// Need config information for each talon
		// Should probably be part of YAML params for controller
		// set initial mode
		// set PIDF constants for both slots (read from nh params)
		// set close loop ramp rate
		// set voltage compensation rate
		// set soft limits - forward/reverse limits and enables
		// set limit switch config - enable, NO/NC
		// set encoder config / reverse

		//can_talons_[i]->Set(0.0); // Make sure motor is stopped
		// TODO : Grab initial mode from config file?
		// Or maybe set it to disabled and require the higher
		// level controller to request a mode on init?
		//int rc = can_talons_[i]->SetModeSelect(CanTalonSRX::kMode_DutyCycle);
		//if (rc != CTR_OKAY)
		//ROS_WARN("*** setModeSelect() failed with %d", rc);
	}

	// TODO : assert nidec_brushles_names_.size() == nidec_brushles_xxx_channels_.size()
	for (size_t i = 0; i < nidec_brushless_names_.size(); i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << nidec_brushless_names_[i] <<
							  " as PWM channel " << nidec_brushless_pwm_channels_[i] <<
							  " / DIO channel " << nidec_brushless_dio_channels_[i]);
	}
	ROS_INFO_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready.");
}

void FRCRobotSimInterface::read(ros::Duration &/*elapsed_time*/)
{
	// Simulated state is updated in write, so just
	// display it here for debugging

	//printState();
}

void FRCRobotSimInterface::write(ros::Duration &elapsed_time)
{
	// Safety - should be using Talon Sim to control this
	// Maybe do a eStop / enabled check instead?
	//enforceLimits(elapsed_time);

	//ROS_INFO_STREAM_THROTTLE(1,
	//std::endl << std::string(__FILE__) << ":" << __LINE__ <<
	//std::endl << "Command" << std::endl << printCommandHelper());

	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		// If commanded mode changes, copy it over
		// to current state
		hardware_interface::TalonMode new_mode;
		if (talon_command_[joint_id].newMode(new_mode))
			talon_state_[joint_id].setTalonMode(new_mode);
		int slot;
		if (talon_command_[joint_id].slotChanged(slot))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF slot to " << slot << std::endl);
			talon_state_[joint_id].setSlot(slot);
		}

		double p;
		double i;
		double d;
		double f;
		int   iz;
		int   allowable_closed_loop_error;
		double max_integral_accumulator;
		for (int j = 0; j < 2; j++)
		{
			if (talon_command_[joint_id].pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, j))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF slot " << j << " config values" << std::endl);
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
			talon_state_[joint_id].setInvert(invert);
			talon_state_[joint_id].setSensorPhase(sensor_phase);
		}

		// Follower doesn't need to be updated - used the
		// followed talon for state instead
		if (talon_state_[joint_id].getTalonMode() == hardware_interface::TalonMode_Follower)
			continue;

		// Assume instant acceleration for now
		double speed;

		bool speed_changed = talon_command_[joint_id].get(speed);
		if (invert)
			speed = -speed;
		if (speed_changed)
			talon_state_[joint_id].setSetpoint(speed);

		talon_state_[joint_id].setPosition(talon_state_[joint_id].getPosition() + (sensor_phase ? -1 : 1 ) * speed * elapsed_time.toSec());
		talon_state_[joint_id].setSpeed((sensor_phase ? -1 : 1 ) * speed);
	}
	for (std::size_t joint_id = 0; joint_id < num_nidec_brushlesses_; ++joint_id)
	{
		// Assume instant acceleration for now
		const double vel = brushless_command_[joint_id];
		brushless_pos_[joint_id] += vel * elapsed_time.toSec();
		brushless_vel_[joint_id] = vel;
	}
}

void FRCRobotSimInterface::enforceLimits(ros::Duration &period)
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

}  // namespace
