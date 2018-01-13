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
							  "Loading joint " << i << "=" << can_talon_srx_names_[i] <<
							  " as CAN id " << can_talon_srx_can_ids_[i]);
	}

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
	ROS_INFO_STREAM_THROTTLE(1,
			std::endl << std::string(__FILE__) << ":" << __LINE__ <<
			std::endl << "Command" << std::endl << printCommandHelper());

	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];
		// If commanded mode changes, copy it over
		// to current state
		hardware_interface::TalonMode new_mode;
		if (tc.newMode(new_mode))
			ts.setTalonMode(new_mode);
		int slot;
		if (tc.slotChanged(slot))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" PIDF slot to " << slot << std::endl);
			ts.setSlot(slot);
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
			if (tc.pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, j))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" PIDF slot " << j << " config values" << std::endl);
				ts.setPidfP(p, j);
				ts.setPidfI(i, j);
				ts.setPidfD(d, j);
				ts.setPidfF(f, j);
				ts.setPidfIzone(iz, j);
				ts.setAllowableClosedLoopError(allowable_closed_loop_error, j);
				ts.setMaxIntegralAccumulator(max_integral_accumulator, j);

			}
		}
		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" invert / phase" << std::endl);
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
		}

		hardware_interface::NeutralMode neutral_mode;
		if (tc.neutralModeChanged(neutral_mode))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral mode" << std::endl);
			ts.setNeutralMode(neutral_mode);
		}

		if (tc.neutralOutputChanged())
		{
			ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral output" << std::endl);
			ts.setNeutralOutput(true);
		}

		double iaccum;
		if (tc.integralAccumulatorChanged(iaccum))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" integral accumulator" << std::endl);
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
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" output shaping" << std::endl);
			ts.setOpenloopRamp(open_loop_ramp);
			ts.setClosedloopRamp(closed_loop_ramp);
			ts.setPeakOutputForward(peak_output_forward);
			ts.setPeakOutputReverse(peak_output_reverse);
			ts.setNominalOutputForward(nominal_output_forward);
			ts.setNominalOutputReverse(nominal_output_reverse);
		}
		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.VoltageCompensationChanged(v_c_saturation,
									v_measurement_filter,
									v_c_enable))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" voltage compensation" << std::endl);
			ts.setVoltageCompensationSaturation(v_c_saturation);
			ts.setVoltageMeasurementFilter(v_measurement_filter);
			ts.setVoltageCompensationEnable(v_c_enable);
		}

		hardware_interface::LimitSwitchSource internal_local_forward_source;
		hardware_interface::LimitSwitchNormal internal_local_forward_normal;
		hardware_interface::LimitSwitchSource internal_local_reverse_source;
		hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
		if (tc.limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
				internal_local_reverse_source, internal_local_reverse_normal))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" limit switches" << std::endl);
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
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" soft limits" << std::endl);
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
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" peak current" << std::endl);
			ts.setPeakCurrentLimit(peak_amps);
			ts.setPeakCurrentDuration(peak_msec);
			ts.setContinuousCurrentLimit(continuous_amps);
			ts.setCurrentLimitEnable(enable);
		}

		double motion_cruise_velocity;
		double motion_acceleration;
		if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" cruise velocity / acceleration" << std::endl);
			ts.setMotionCruiseVelocity(motion_cruise_velocity);
			ts.setMotionAcceleration(motion_acceleration);
		}
		// Do this before rest of motion profile stuff
		// so it takes effect before starting a buffer?
		int motion_control_frame_period;
		if (tc.motionControlFramePeriodChanged(motion_control_frame_period))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion control frame period" << std::endl);
			ts.setMotionControlFramePeriod(motion_control_frame_period);
		}

		if (tc.clearMotionProfileTrajectoriesChanged())
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories" << std::endl);

		if (tc.clearMotionProfileHasUnderrunChanged())
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile underrun changed" << std::endl);

		std::vector<hardware_interface::TrajectoryPoint> trajectory_points;

		if (tc.motionProfileTrajectoriesChanged(trajectory_points))
			ROS_INFO_STREAM("Added joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories" << std::endl);

		if (ts.getTalonMode() == hardware_interface::TalonMode_Position)
		{
			// Assume instant velocity
			double position;

			const bool position_changed = tc.commandChanged(position);
			if (invert)
				position = -position;
			if (position_changed)
				ts.setSetpoint(position);

			ts.setPosition((sensor_phase ? -1 : 1 ) * position);
			ts.setSpeed(0);
		}
		else if (ts.getTalonMode() == hardware_interface::TalonMode_Velocity)
		{
			// Assume instant acceleration for now
			double speed;

			const bool speed_changed = tc.commandChanged(speed);
			if (invert)
				speed = -speed;
			if (speed_changed)
				ts.setSetpoint(speed);

			ts.setPosition(ts.getPosition() + (sensor_phase ? -1 : 1 ) * speed * elapsed_time.toSec());
			ts.setSpeed((sensor_phase ? -1 : 1 ) * speed);
		}
	}
	for (std::size_t joint_id = 0; joint_id < num_nidec_brushlesses_; ++joint_id)
	{
		// Assume instant acceleration for now
		const double vel = brushless_command_[joint_id];
		brushless_pos_[joint_id] += vel * elapsed_time.toSec();
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
		if (setpoint >= 1.)
			setpoint = 1.;
		else if (setpoint <= -1.)
			setpoint = -1.;
		else
			setpoint = 0.;

		double_solenoid_state_[i] = setpoint;
	}
}

}  // namespace
