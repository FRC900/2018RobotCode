#pragma once

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{
// These should mirror the modes listed in ControlModes.h
// Need a separate copy here though, since sim won't be
// including that header file - sim shouldn't require
// anything specifically from the actual motor controller
// hardware
enum TalonMode
{
	TalonMode_Uninitialized = -1,
	TalonMode_PercentOutput,
	TalonMode_Position,      // CloseLoop
	TalonMode_Velocity,      // CloseLoop
	TalonMode_Current,       // CloseLoop
	TalonMode_Follower,
	TalonMode_MotionProfile,
	TalonMode_MotionMagic,
	TalonMode_Disabled,
	TalonMode_Last
};

enum NeutralMode
{
	NeutralMode_Uninitialized,
	NeutralMode_EEPROM_Setting,
	NeutralMode_Coast,
	NeutralMode_Brake,
	NeutralMode_Last
};

enum FeedbackDevice
{
	FeedbackDevice_Uninitialized,
	FeedbackDevice_QuadEncoder,
	FeedbackDevice_CTRE_MagEncoder_Relative = FeedbackDevice_QuadEncoder,
	FeedbackDevice_Analog,
	FeedbackDevice_Tachometer,
	FeedbackDevice_PulseWidthEncodedPosition,
	FeedbackDevice_CTRE_MagEncoder_Absolute = FeedbackDevice_PulseWidthEncodedPosition,
	FeedbackDevice_SensorSum,
	FeedbackDevice_SensorDifference,
	FeedbackDevice_RemoteSensor0,
	FeedbackDevice_RemoteSensor1,
	FeedbackDevice_SoftwareEmulatedSensor,
	FeedbackDevice_Last
};

enum LimitSwitchSource
{
	LimitSwitchSource_Uninitialized,
	LimitSwitchSource_FeedbackConnector,
	LimitSwitchSource_RemoteTalonSRX,
	LimitSwitchSource_RemoteCANifier,
	LimitSwitchSource_Deactivated,
	LimitSwitchSource_Last
};

enum RemoteLimitSwitchSource
{
	RemoteLimitSwitchSource_Uninitialized,
	RemoteLimitSwitchSource_RemoteTalonSRX,
	RemoteLimitSwitchSource_RemoteCANifier,
	RemoteLimitSwitchSource_Deactivated,
	RemoteLimitSwitchSource_Last
};

enum LimitSwitchNormal
{
	LimitSwitchNormal_Uninitialized,
	LimitSwitchNormal_NormallyOpen,
	LimitSwitchNormal_NormallyClosed,
	LimitSwitchNormal_Disabled,
	LimitSwitchNormal_Last
};


// Match up with CTRE Motion profile struct
enum SetValueMotionProfile
{
	Disable = 0, Enable = 1, Hold = 2,
};

struct MotionProfileStatus
{
	int  topBufferRem;
	int  topBufferCnt;
	int  btmBufferCnt;
	bool hasUnderrun;
	bool isUnderrun;
	bool activePointValid;
	bool isLast;
	int  profileSlotSelect0;
	int  profileSlotSelect1;
	SetValueMotionProfile outputEnable;
	int  timeDurMs;

	MotionProfileStatus(void):
		topBufferRem(0),
		topBufferCnt(0),
		btmBufferCnt(0),
		hasUnderrun(false),
		isUnderrun(false),
		activePointValid(false),
		isLast(false),
		profileSlotSelect0(0),
		profileSlotSelect1(0),
		outputEnable(Disable),
		timeDurMs(0)
	{
	}

};

// Class which contains state information
// about a given Talon SRX. This should include
// data about the mode the Talon is running in,
// current config and setpoint as well as data
// from the attached encoders, limit switches,
// etc.
// Each pass through read() in the low-level
// hardware interface should update the member
// vars of this class.
// The controllers can access the member variables
// as needed to make decisions in their update code
// The hardware_controller is responsible for keeping
// a master array of these classes - 1 entry per
// physical Talon controller in the robot
class TalonHWState
{
	public:
		TalonHWState(int can_id) :
			setpoint_(0.),
			position_(0),
			speed_(0),
			output_voltage_(0),
			output_current_(0),
			bus_voltage_(0),
			motor_output_percent_(0),
			temperature_(0),
			pidf_p_ {0, 0},
			pidf_i_ {0, 0},
			pidf_d_ {0, 0},
			pidf_f_ {0, 0},
			pidf_izone_ {0, 0},
			allowable_closed_loop_error_ {0, 0},
			max_integral_accumulator_ {0, 0},
			closed_loop_peak_output_{1, 1},
			closed_loop_period_{20, 20},
			aux_pid_polarity_(false),
			closed_loop_error_(0.0),
			integral_accumulator_(0.0),
			error_derivative_(0.0),
			closed_loop_target_(0.0),
			active_trajectory_position_(0.0),
			active_trajectory_velocity_(0.0),
			active_trajectory_heading_(0.0),
			forward_limit_switch_closed_(false),
			reverse_limit_switch_closed_(false),
			forward_softlimit_hit_(false),
			reverse_softlimit_hit_(false),
			talon_mode_(TalonMode_Uninitialized),
			can_id_(can_id),
			slot_(0),
			invert_(false),
			sensor_phase_(false),
			neutral_mode_(NeutralMode_Uninitialized),
			neutral_output_(false),
			encoder_feedback_(FeedbackDevice_Uninitialized),
			feedback_coefficient_(1.0),
			encoder_ticks_per_rotation_(4096),

			//output shaping
			close_loop_ramp_(0),
			open_loop_ramp_(0),
			peak_output_forward_(1.),
			peak_output_reverse_(-1.),
			nominal_output_forward_(0.),
			nominal_output_reverse_(0.),
			neutral_deadband_(0.),

			// voltage compensation
			voltage_compensation_saturation_(12.5),
			voltage_measurement_filter_(32),
			voltage_compensation_enable_(true),

			limit_switch_local_forward_source_(LimitSwitchSource_FeedbackConnector),
			limit_switch_local_forward_normal_(LimitSwitchNormal_NormallyOpen),
			limit_switch_local_reverse_source_(LimitSwitchSource_FeedbackConnector),
			limit_switch_local_reverse_normal_(LimitSwitchNormal_NormallyOpen),
			limit_switch_remote_forward_source_(RemoteLimitSwitchSource_Deactivated),
			limit_switch_remote_forward_normal_(LimitSwitchNormal_NormallyOpen),
			limit_switch_remote_reverse_source_(RemoteLimitSwitchSource_Deactivated),
			limit_switch_remote_reverse_normal_(LimitSwitchNormal_NormallyOpen),

			// soft limits
			softlimit_forward_threshold_(0.0),
			softlimit_forward_enable_(false),
			softlimit_reverse_threshold_(0.0),
			softlimit_reverse_enable_(false),
			softlimits_override_enable_(false),

			// current limiting
			current_limit_peak_amps_(0),
			current_limit_peak_msec_(0),
			current_limit_continuous_amps_(0),
			current_limit_enable_(false),

			motion_cruise_velocity_(0),
			motion_acceleration_(0),

			// motion profiling
			motion_profile_top_level_buffer_count_(0),
			motion_profile_top_level_buffer_full_(false),
			motion_control_frame_period_(20), // Guess at 50Hz?
			motion_profile_trajectory_period_(0),

			// faults
			faults_(0),
			sticky_faults_(0),

			conversion_factor_(1.0)
		{
		}
		
		double getSetpoint(void) const
		{
			return setpoint_;
		}
		double getPosition(void) const
		{
			return position_;
		}
		double getSpeed(void) const
		{
			return speed_;
		}
		double getOutputVoltage(void) const
		{
			return output_voltage_;
		}
		int    getCANID(void) const
		{
			return can_id_;
		}
		double getOutputCurrent(void) const
		{
			return output_current_;
		}
		double getBusVoltage(void) const
		{
			return bus_voltage_;
		}
		double getMotorOutputPercent(void) const
		{
			return motor_output_percent_;
		}
		double getTemperature(void) const
		{
			return temperature_;
		}
		double getPidfP(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return pidf_p_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;
			}
		}
		double getPidfI(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return pidf_i_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;
			}
		}
		double getPidfD(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return pidf_d_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;
			}
		}
		double getPidfF(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return pidf_f_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;
			}
		}
		int getPidfIzone(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return pidf_izone_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0;
			}
		}
		int getAllowableClosedLoopError(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return allowable_closed_loop_error_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0;
			}
		}
		double getMaxIntegralAccumulator(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return max_integral_accumulator_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0;
			}
		}
		double getClosedLoopPeakOutput(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return closed_loop_peak_output_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0;
			}
		}
		int getClosedLoopPeriod(size_t index) const
		{
			if ((index == 0) || (index == 1))
				return closed_loop_period_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0;
			}
		}
		bool getAuxPidPolarity(void) const
		{
			return aux_pid_polarity_;
		}

		double getClosedLoopError(void) const
		{
			return closed_loop_error_;
		}
		double getIntegralAccumulator(void) const
		{
			return integral_accumulator_;
		}
		double getErrorDerivative(void) const
		{
			return error_derivative_;
		}
		double getClosedLoopTarget(void) const
		{
			return closed_loop_target_;
		}
		double getActiveTrajectoryPosition(void) const
		{
			return active_trajectory_position_;
		}
		double getActiveTrajectoryVelocity(void) const
		{
			return active_trajectory_velocity_;
		}
		double getActiveTrajectoryHeading(void) const
		{
			return active_trajectory_heading_;
		}
		bool getForwardLimitSwitch(void) const
		{
			return forward_limit_switch_closed_;
		}
		bool getReverseLimitSwitch(void) const
		{
			return reverse_limit_switch_closed_;
		}
		bool getForwardSoftlimitHit(void) const
		{
			return forward_softlimit_hit_;
		}
		bool getReverseSoftlimitHit(void) const
		{
			return reverse_softlimit_hit_;
		}

		TalonMode getTalonMode(void) const
		{
			return talon_mode_;
		}

		int  getSlot(void) const
		{
			return slot_;
		}
		bool getInvert(void) const
		{
			return invert_;
		}
		bool getSensorPhase(void) const
		{
			return sensor_phase_;
		}

		NeutralMode getNeutralMode(void) const
		{
			return neutral_mode_;
		}
		bool getNeutralOutput(void) const
		{
			return neutral_output_;
		}
		FeedbackDevice getEncoderFeedback(void) const
		{
			return encoder_feedback_;
		}
		double getFeedbackCoefficient(void) const
		{
			return feedback_coefficient_;
		}
		int getEncoderTicksPerRotation(void) 	const
		{
			return encoder_ticks_per_rotation_;
		}

		unsigned int getFaults(void) const
		{
			return faults_;
		}
		unsigned int getStickyFaults(void) const
		{
			return sticky_faults_;
		}
		double getConversionFactor(void) const
		{
			return conversion_factor_;
		}
		void setConversionFactor(double conversion_factor)
		{
			conversion_factor_ = conversion_factor;
		}		
		void setSetpoint(double setpoint)
		{
			setpoint_ = setpoint;
		}
		void setPosition(double position)
		{
			position_ = position;
		}
		void setSpeed(double speed)
		{
			speed_ = speed;
		}
		void setOutputVoltage(double output_voltage)
		{
			output_voltage_ = output_voltage;
		}
		void setOutputCurrent(double output_current)
		{
			output_current_ = output_current;
		}
		void setBusVoltage(double bus_voltage)
		{
			bus_voltage_ = bus_voltage;
		}
		void setMotorOutputPercent(double motor_output_percent)
		{
			motor_output_percent_ = motor_output_percent;
		}
		void setTemperature(double temperature)
		{
			temperature_ = temperature;
		}

		//output shaping
		void setClosedloopRamp(double close_loop_ramp)
		{
			close_loop_ramp_ = close_loop_ramp;
		}
		double getClosedloopRamp(void) const
		{
			return close_loop_ramp_;
		}

		void setOpenloopRamp(double open_loop_ramp)
		{
			open_loop_ramp_ = open_loop_ramp;
		}
		double getOpenloopRamp(void) const
		{
			return open_loop_ramp_;
		}

		void setPeakOutputForward(double peak_output_forward)
		{
			peak_output_forward_ = peak_output_forward;
		}
		double getPeakOutputForward(void) const
		{
			return peak_output_forward_;
		}
		void setPeakOutputReverse(double peak_output_reverse)
		{
			peak_output_reverse_ = peak_output_reverse;
		}
		double getPeakOutputReverse(void) const
		{
			return peak_output_reverse_;
		}

		void setNominalOutputForward(double nominal_output_forward)
		{
			nominal_output_forward_ = nominal_output_forward;
		}
		double getNominalOutputForward(void) const
		{
			return nominal_output_forward_;
		}
		void setNominalOutputReverse(double nominal_output_reverse)
		{
			nominal_output_reverse_ = nominal_output_reverse;
		}
		double getNominalOutputReverse(void) const
		{
			return nominal_output_reverse_;
		}

		void setNeutralDeadband(double neutral_deadband)
		{
			neutral_deadband_ = neutral_deadband;
		}
		double getNeutralDeadband(void) const
		{
			return neutral_deadband_;
		}

		void setVoltageCompensationSaturation(double voltage_compensation_saturation)
		{
			voltage_compensation_saturation_ = voltage_compensation_saturation;
		}
		double getVoltageCompensationSaturation(void) const
		{
			return voltage_compensation_saturation_;
		}

		void setVoltageMeasurementFilter(int voltage_measurement_filter)
		{
			voltage_measurement_filter_ = voltage_measurement_filter;
		}
		int getVoltageMeasurementFilter(void) const
		{
			return voltage_measurement_filter_;
		}

		void setVoltageCompensationEnable(bool voltage_compensation_enable)
		{
			voltage_compensation_enable_ = voltage_compensation_enable;
		}
		bool getVoltageCompensationEnable(void) const
		{
			return voltage_compensation_enable_;
		}

		void setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
		{
			limit_switch_local_forward_source_ = source;
			limit_switch_local_forward_normal_ = normal;
		}

		void getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_forward_source_;
			normal = limit_switch_local_forward_normal_;
		}

		void setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
		{
			limit_switch_local_reverse_source_ = source;
			limit_switch_local_reverse_normal_ = normal;
		}

		void getReverseLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_reverse_source_;
			normal = limit_switch_local_reverse_normal_;
		}

		void setForwardSoftLimitThreshold(double threshold)
		{
			softlimit_forward_threshold_ = threshold;
		}
		double getForwardSoftLimitThreshold(void) const
		{
			return softlimit_forward_threshold_;
		}

		void setForwardSoftLimitEnable(bool enable)
		{
			softlimit_forward_enable_ = enable;
		}
		bool getForwardSoftLimitEnable(void) const
		{
			return softlimit_forward_enable_;
		}
		void setReverseSoftLimitThreshold(double threshold)
		{
			softlimit_reverse_threshold_ = threshold;
		}
		double getReverseSoftLimitThreshold(void) const
		{
			return softlimit_reverse_threshold_;
		}

		void setReverseSoftLimitEnable(bool enable)
		{
			softlimit_reverse_enable_ = enable;
		}
		bool getReverseSoftLimitEnable(void) const
		{
			return softlimit_reverse_enable_;
		}

		void setOverrideSoftLimitsEnable(bool enable)
		{
			softlimits_override_enable_ = enable;
		}
		bool getOverrideSoftLimitsEnable(void) const
		{
			return softlimits_override_enable_;
		}

		void setPeakCurrentLimit(int amps)
		{
			current_limit_peak_amps_ = amps;
		}
		int getPeakCurrentLimit(void) const
		{
			return current_limit_peak_amps_;
		}

		void setPeakCurrentDuration(int msec)
		{
			current_limit_peak_msec_ = msec;
		}
		int getPeakCurrentDuration(void) const
		{
			return current_limit_peak_msec_;
		}
		void setContinuousCurrentLimit(int amps)
		{
			current_limit_continuous_amps_ = amps;
		}
		int getContinuousCurrentLimit(void) const
		{
			return current_limit_continuous_amps_;
		}
		void setCurrentLimitEnable(bool enable)
		{
			current_limit_enable_ = enable;
		}
		bool getCurrentLimitEnable(void) const
		{
			return current_limit_enable_;
		}

		void setMotionCruiseVelocity(double velocity)
		{
			motion_cruise_velocity_ = velocity;
		}
		double getMotionCruiseVelocity(void) const
		{
			return motion_cruise_velocity_;
		}
		void setMotionAcceleration(double acceleration)
		{
			motion_acceleration_ = acceleration;
		}
		double getMotionAcceleration(void) const
		{
			return motion_acceleration_;
		}

		void setMotionProfileTopLevelBufferCount(int count)
		{
			motion_profile_top_level_buffer_count_ = count;
		}
		int getMotionProfileTopLevelBufferCount(void) const
		{
			return motion_profile_top_level_buffer_count_;
		}
		void setMotionProfileTopLevelBufferFull(bool is_full)
		{
			motion_profile_top_level_buffer_full_ = is_full;
		}
		bool getMotionProfileTopLevelBufferFull(void) const
		{
			return motion_profile_top_level_buffer_full_;
		}
		void setMotionProfileStatus(const MotionProfileStatus &status)
		{
			motion_profile_status_ = status;
		}
		MotionProfileStatus getMotionProfileStatus(void) const
		{
			return motion_profile_status_;
		}
		void setMotionControlFramePeriod(int msec)
		{
			motion_control_frame_period_ = msec;
		}
		int getMotionControlFramePeriod(void) const
		{
			return motion_control_frame_period_;
		}
		void setMotionProfileTrajectoryPeriod(int msec)
		{
			motion_profile_trajectory_period_ = msec;
		}
		int getMotionProfileTrajectoryPeriod(void) const
		{
			return motion_profile_trajectory_period_;
		}

		void setPidfP(double pidf_p, size_t index)
		{
			if ((index == 0) || (index == 1))
				pidf_p_[index] = pidf_p;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setPidfI(double pidf_i, size_t index)
		{
			if ((index == 0) || (index == 1))
				pidf_i_[index] = pidf_i;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setPidfD(double pidf_d, size_t index)
		{
			if ((index == 0) || (index == 1))
				pidf_d_[index] = pidf_d;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setPidfF(double pidf_f, size_t index)
		{
			if ((index == 0) || (index == 1))
				pidf_f_[index] = pidf_f;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setPidfIzone(int pidf_izone, size_t index)
		{
			if ((index == 0) || (index == 1))
				pidf_izone_[index] = pidf_izone;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setAllowableClosedLoopError(int allowable_closed_loop_error, size_t index)
		{
			if ((index == 0) || (index == 1))
				allowable_closed_loop_error_[index] = allowable_closed_loop_error;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setMaxIntegralAccumulator(double max_integral_accumulator, size_t index)
		{
			if ((index == 0) || (index == 1))
				max_integral_accumulator_[index] = max_integral_accumulator;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index)
		{
			if ((index == 0) || (index == 1))
				closed_loop_peak_output_[index] = closed_loop_peak_output;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setClosedLoopPeriod(double closed_loop_period, size_t index)
		{
			if ((index == 0) || (index == 1))
				closed_loop_period_[index] = closed_loop_period;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
		}
		void setAuxPidPolarity(bool aux_pid_polarity)
		{
			aux_pid_polarity_ = aux_pid_polarity;
		}

		void setClosedLoopError(double closed_loop_error)
		{
			closed_loop_error_ = closed_loop_error;
		}
		void setIntegralAccumulator(double integral_accumulator)
		{
			integral_accumulator_ = integral_accumulator;
		}
		void setErrorDerivative(double error_derivative)
		{
			error_derivative_ = error_derivative;
		}
		void setClosedLoopTarget(double closed_loop_target)
		{
			closed_loop_target_ = closed_loop_target;
		}
		void setActiveTrajectoryPosition(double active_trajectory_position)
		{
			active_trajectory_position_ = active_trajectory_position;
		}
		void setActiveTrajectoryVelocity(double active_trajectory_velocity)
		{
			active_trajectory_velocity_ = active_trajectory_velocity;
		}
		void setActiveTrajectoryHeading(double active_trajectory_heading)
		{
			active_trajectory_heading_ = active_trajectory_heading;
		}
		void setForwardLimitSwitch(bool forward_limit_switch_closed)
		{
			forward_limit_switch_closed_ = forward_limit_switch_closed;
		}
		void setReverseLimitSwitch(bool reverse_limit_switch_closed)
		{
			reverse_limit_switch_closed_ = reverse_limit_switch_closed;
		}
		void setForwardSoftlimitHit(bool forward_softlimit_hit)
		{
			forward_softlimit_hit_ = forward_softlimit_hit;
		}
		void setReverseSoftlimitHit(bool reverse_softlimit_hit)
		{
			reverse_softlimit_hit_ = reverse_softlimit_hit;
		}

		void setTalonMode(TalonMode talon_mode)
		{
			if ((talon_mode_ >= TalonMode_Uninitialized) &&
				(talon_mode_ <  TalonMode_Last) )
				talon_mode_ = talon_mode;
			else
				ROS_WARN_STREAM("Invalid talon mode requested");
		}
		void setSlot(int slot)
		{
			slot_ = slot;
		}
		void setInvert(bool invert)
		{
			invert_ = invert;
		}
		void setSensorPhase(bool sensor_phase)
		{
			sensor_phase_ = sensor_phase;
		}
		void setNeutralMode(NeutralMode neutral_mode)
		{
			if ((neutral_mode_ >= NeutralMode_Uninitialized) &&
				(neutral_mode_ <  NeutralMode_Last) )
				neutral_mode_ = neutral_mode;
			else
				ROS_WARN_STREAM("Invalid neutral mode requested");
		}
		void setNeutralOutput(bool neutral_output)
		{
			neutral_output_ = neutral_output;
		}

		void setEncoderFeedback(FeedbackDevice encoder_feedback)
		{
			if ((encoder_feedback >= FeedbackDevice_Uninitialized) &&
				(encoder_feedback <  FeedbackDevice_Last) )
				encoder_feedback_ = encoder_feedback;
			else
				ROS_WARN_STREAM("Invalid feedback device requested");
		}
		void setFeedbackCoefficient(double feedback_coefficient)
		{
			feedback_coefficient_ = feedback_coefficient;
		}
		void setEncoderTicksPerRotation(int encoder_ticks_per_rotation)
		{
			encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
		}
		void setFaults(unsigned int faults)
		{
			faults_ = faults;
		}
		void setStickyFaults(unsigned int sticky_faults)
		{
			sticky_faults_ = sticky_faults;
		}

	private:
		double setpoint_;
		double position_;
		double speed_;
		double output_voltage_;
		double output_current_;
		double bus_voltage_;
		double motor_output_percent_;
		double temperature_;
		double pidf_p_[2];
		double pidf_i_[2];
		double pidf_d_[2];
		double pidf_f_[2];
		int   pidf_izone_[2];
		int   allowable_closed_loop_error_[2];
		double max_integral_accumulator_[2];
		double closed_loop_peak_output_[2];
		int    closed_loop_period_[2];
		bool   aux_pid_polarity_;
		double closed_loop_error_;
		double integral_accumulator_;
		double error_derivative_;
		double closed_loop_target_;
		double active_trajectory_position_;
		double active_trajectory_velocity_;
		double active_trajectory_heading_;
		bool forward_limit_switch_closed_;
		bool reverse_limit_switch_closed_;
		bool forward_softlimit_hit_;
		bool reverse_softlimit_hit_;
		TalonMode talon_mode_;

		int can_id_;

		int slot_;
		bool invert_;
		bool sensor_phase_;

		NeutralMode neutral_mode_;
		bool        neutral_output_;

		FeedbackDevice encoder_feedback_;
		double feedback_coefficient_;
		int encoder_ticks_per_rotation_;

		// output shaping
		double close_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;

		// voltage compensation
		double voltage_compensation_saturation_;
		int   voltage_measurement_filter_;
		bool  voltage_compensation_enable_;

		LimitSwitchSource limit_switch_local_forward_source_;
		LimitSwitchNormal limit_switch_local_forward_normal_;
		LimitSwitchSource limit_switch_local_reverse_source_;
		LimitSwitchNormal limit_switch_local_reverse_normal_;
		RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		LimitSwitchNormal limit_switch_remote_forward_normal_;
		RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		LimitSwitchNormal limit_switch_remote_reverse_normal_;

		// soft limits
		double softlimit_forward_threshold_;
		bool softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool softlimit_reverse_enable_;
		bool softlimits_override_enable_;

		// current limiting
		int current_limit_peak_amps_;
		int current_limit_peak_msec_;
		int current_limit_continuous_amps_;
		bool current_limit_enable_;

		// Talon expects these in integral sensorUnitsPer100ms,
		// but at this level we're still dealing with
		// radians/sec (or /sec^2 for acceleration)
		double motion_cruise_velocity_;
		double motion_acceleration_;

		// Motion profiling
		int motion_profile_top_level_buffer_count_;
		bool motion_profile_top_level_buffer_full_;
		MotionProfileStatus motion_profile_status_;
		int motion_control_frame_period_;
		int motion_profile_trajectory_period_;

		unsigned int faults_;
		unsigned int sticky_faults_;

		double conversion_factor_;
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to get state
// info about a Talon
class TalonStateHandle
{
	public:
		TalonStateHandle(void) :
			state_(0)
		{}

		// Initialize the base JointStateHandle with pointers
		// from the state data object.  Since the standard ROS
		// code uses JointStateHandles in some places to display
		// robot state support that code as much as possible.  We'll
		// have to figure out what effort maps to in the Talon
		// Anything above and beyond the 3 standard ROS state
		// vars (position, velocity, effort) will require support
		// in the controller as well as the HWState object pointed
		// to by a given handle.
		TalonStateHandle(const std::string &name, const TalonHWState *state) :
			name_(name),
			state_(state)
		{
			if (!state)
				throw HardwareInterfaceException("Cannot create Talon state handle '" + name + "'. state pointer is null.");
		}
		std::string getName(void) const
		{
			return name_;
		}

		// Operator which allows access to methods from
		// the TalonHWState member var associated with this
		// handle
		// Note that we could create separate methods in
		// the handle class for every method in the HWState
		// class, e.g.
		//     double getFoo(void) const {assert(_state); return state_->getFoo();}
		// but if each of them just pass things unchanged between
		// the calling code and the HWState method there's no
		// harm in making a single method to do so rather than
		// dozens of getFoo() one-line methods
		const TalonHWState *operator->() const
		{
			assert(state_);
			return state_;
		}

	private:
		std::string         name_;
		const TalonHWState *state_; // leave this const since state should never change the Talon itself
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
class TalonStateInterface : public HardwareResourceManager<TalonStateHandle> {};
}
