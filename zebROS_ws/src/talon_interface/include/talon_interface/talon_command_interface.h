#pragma once

#include <cassert>
#include <string>
#include <talon_interface/talon_state_interface.h>

namespace hardware_interface
{
enum TrajectoryDuration 
{
	TrajectoryDuration_0ms = 0,
	TrajectoryDuration_5ms = 5,
	TrajectoryDuration_10ms = 10,
	TrajectoryDuration_20ms = 20,
	TrajectoryDuration_30ms = 30,
	TrajectoryDuration_40ms = 40,
	TrajectoryDuration_50ms = 50,
	TrajectoryDuration_100ms = 100,
};

//Below struct contains all the info we want for our profile
//Later it might include some more complex settings (current limits?, peak output limits?, 
//some f params to calc on the fly based on sensor data?)
struct CustomProfilePoint
{
	CustomProfilePoint() :
		positionMode(true), //versus velocity mode
		pidSlot(0),
		setpoint(0.0),
		fTerm(0),
		duration(0),
		zeroPos(false)
	{
	}
	bool positionMode;
	int pidSlot;
	double setpoint;
	double fTerm;
	double duration;
	bool zeroPos;
};

	
struct TrajectoryPoint
{
	// Sane? defaults
	TrajectoryPoint() :
		position(0),
		velocity(0),
		headingRad(0),
		auxiliaryPos(0),
		profileSlotSelect0(0),
		profileSlotSelect1(0),
		isLastPoint(false),
		zeroPos(false),
		trajectoryDuration(TrajectoryDuration::TrajectoryDuration_0ms)
	{
	}
	double position;
	double velocity;
	double headingRad;
	double auxiliaryPos;
	uint32_t profileSlotSelect0;
	uint32_t profileSlotSelect1;
	bool isLastPoint;
	bool zeroPos;
	TrajectoryDuration trajectoryDuration;
};

// Class to buffer data needed to set the state of the
// Talon.  This should (eventually) include anything
// which might be set during runtime.  Config data
// which is set only once at startup can be handled
// in the hardware manager constructor/init rather than through
// this interface.
// Various controller code will set the member vars of
// this class depending on the needs of the motor
// being controlled
// Each pass through write() in the hardware interface
// will use this to re-configure (if necessary) and then
// update the setpoint on the associated Talon.
// The hardware_controller is responsible for keeping
// a master array of these classes - 1 entry per
// physical Talon controller in the robot
class TalonHWCommand
{
	public:
		// Set up default values
		// Set most of the changed_ vars to true
		// to force a write of these values to the Talon
		// That should put the talon in a known state
		// rather than relying on them being setup to
		// a certain state previously
		TalonHWCommand(void) :
			command_(0),
			command_changed_(true),
			mode_(TalonMode_Uninitialized),
			mode_changed_(false),
			demand1_type_(DemandType_Neutral),
			demand1_value_(0.0),
			demand1_changed_(true),
			pidf_slot_(0),
			pidf_slot_changed_(true),
			iaccum_(0.0),
			iaccum_changed_(false),
			invert_(false),
			sensor_phase_(false),
			invert_changed_(true),
			neutral_mode_(NeutralMode_Uninitialized),
			neutral_mode_changed_(false),
			neutral_output_(false),
			encoder_feedback_(FeedbackDevice_Uninitialized),
			feedback_coefficient_(1.0),
			encoder_feedback_changed_(false),
			encoder_ticks_per_rotation_(4096),

			//output shaping
			closed_loop_ramp_(0),
			open_loop_ramp_(0),
			peak_output_forward_(1.),
			peak_output_reverse_(-1.),
			nominal_output_forward_(0.),
			nominal_output_reverse_(0.),
			neutral_deadband_(0.04),
			output_shaping_changed_(true),

			// voltage compensation
			voltage_compensation_saturation_(12.5), //max voltage to apply to talons when command is 100%
			voltage_measurement_filter_(32), //number of samples in the average of voltage measurements
			voltage_compensation_enable_(true),
			voltage_compensation_changed_(true),

			velocity_measurement_period_(Period_100Ms),
			velocity_measurement_window_(64),
			velocity_measurement_changed_(true),

			sensor_position_value_(0.),
			sensor_position_changed_(false),

			// limit switches
			limit_switch_local_forward_source_(LimitSwitchSource_FeedbackConnector),
			limit_switch_local_forward_normal_(LimitSwitchNormal_Disabled),
			limit_switch_local_reverse_source_(LimitSwitchSource_FeedbackConnector),
			limit_switch_local_reverse_normal_(LimitSwitchNormal_Disabled),
			limit_switch_local_changed_(true),
			limit_switch_remote_forward_source_(RemoteLimitSwitchSource_Deactivated),
			limit_switch_remote_forward_normal_(LimitSwitchNormal_Disabled),
			limit_switch_remote_reverse_source_(RemoteLimitSwitchSource_Deactivated),
			limit_switch_remote_reverse_normal_(LimitSwitchNormal_Disabled),
			limit_switch_remote_changed_(true),

			// soft limits
			softlimit_forward_threshold_(0.0),
			softlimit_forward_enable_(false),
			softlimit_reverse_threshold_(0.0),
			softlimit_reverse_enable_(false),
			softlimits_override_enable_(true),
			softlimit_changed_(true),

			// current limiting
			current_limit_peak_amps_(0),
			current_limit_peak_msec_(10), // see https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/README.md#motor-output-direction-is-incorrect-or-accelerates-when-current-limit-is-enabled
			current_limit_continuous_amps_(0),
			current_limit_enable_(false),
			current_limit_changed_(true),

			motion_cruise_velocity_(0),
			motion_acceleration_(0),
			motion_cruise_changed_(true),

			motion_profile_clear_trajectories_(false),
			motion_profile_clear_has_underrun_(false),
			motion_profile_control_frame_period_(20),
			motion_profile_control_frame_period_changed_(true),
			motion_profile_profile_trajectory_period_(0),
			motion_profile_profile_trajectory_period_changed_(true),

			clear_sticky_faults_(false),
			p_{0, 0},
			i_{0, 0},
			d_{0, 0},
			f_{0, 0},
			i_zone_{0, 0},
			allowable_closed_loop_error_{0, 0}, // need better defaults
			max_integral_accumulator_{0, 0},
			closed_loop_peak_output_{1, 1},
			closed_loop_period_{1, 1},
			aux_pid_polarity_(false),
			aux_pid_polarity_changed_(true),
			pidf_changed_{true, true},

			conversion_factor_(1.0),
			conversion_factor_changed_(true),
			
			custom_profile_run_(false),
			custom_profile_slot_(0),
			custom_profile_hz_(50.0)

		{
			//This is a bit dirty...
			//I apologize for my problems spilling out over your nice and clean code, Kevin
			int garbage;
			char** more_garbage;
			ros::init(garbage, more_garbage, "soon_to_die_talon_command_node");
			ros::NodeHandle n;	
			ros::NodeHandle n_params_behaviors(n, "auto_params");
			int num_profile_slots;

			if (!n_params_behaviors.getParam("num_profile_slots", num_profile_slots))
			ROS_ERROR("Didn't read param num_profile_slots in talon code");
			
			ros::shutdown(); //Now that its short life is over, the mayfly dies and the cycle begins anew

			custom_profile_points_.resize(num_profile_slots); //change as needed
			custom_profile_total_time_.resize(num_profile_slots); 		
		}
		// This gets the requested setpoint, not the
		// status actually read from the controller
		// Need to think about which makes the most
		// sense to query...
		bool commandChanged(double &command)
		{
			command = command_;
			if (!command_changed_)
				return false;
			command_changed_ = false;
			return true;
		}
		double get(void) const
		{
			return command_;
		}

		TalonMode getMode(void) const
		{
			return mode_;
		}

		void setP(double oldP, size_t index)
		{
			if (index >= (sizeof(p_) / sizeof(p_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setP()");
				return;
			}
			if (oldP != p_[index])
			{
				pidf_changed_[index] = true;
				p_[index] = oldP;
			}
		}
		double getP(size_t index) const
		{
			if (index >= (sizeof(p_) / sizeof(p_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getP()");
				return 0.0;
			}
			return p_[index];
		}

		void setI(double ii, size_t index)
		{
			if (index >= (sizeof(i_) / sizeof(i_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setI()");
				return;
			}
			if (ii != i_[index])
			{
				pidf_changed_[index] = true;
				i_[index] = ii;
			}
		}
		double getI(size_t index) const
		{
			if (index >= (sizeof(i_) / sizeof(i_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getI()");
				return 0.0;
			}
			return i_[index];
		}

		void setD(double dd, size_t index)
		{
			if (index >= (sizeof(d_) / sizeof(d_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setD()");
				return;
			}
			if (dd != d_[index])
			{
				pidf_changed_[index] = true;
				d_[index] = dd;
			}
		}
		double getD(size_t index) const
		{
			if (index >= (sizeof(d_) / sizeof(d_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getD()");
				return 0.0;
			}
			return d_[index];
		}

		void setF(double ff, size_t index)
		{
			if (index >= (sizeof(f_) / sizeof(f_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setF()");
				return;
			}
			if (ff != f_[index])
			{
				pidf_changed_[index] = true;
				f_[index] = ff;
			}
		}
		double getF(size_t index)
		{
			if (index >= (sizeof(f_) / sizeof(f_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getF()");
				return 0.0;
			}
			return f_[index];
		}

		void setIZ(int i_zone, size_t index)
		{
			if (index >= (sizeof(i_zone_) / sizeof(i_zone_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setIZ()");
				return;
			}
			if (i_zone != i_zone_[index])
			{
				pidf_changed_[index] = true;
				i_zone_[index] = i_zone;
			}
		}
		int getIZ(size_t index) const
		{
			if (index >= (sizeof(i_zone_) / sizeof(i_zone_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getIZ()");
				return 0.0;
			}
			return i_zone_[index];
		}

		void setAllowableClosedloopError(int allowable_closed_loop_error, size_t index)
		{
			if (index >= (sizeof(allowable_closed_loop_error_) / sizeof(allowable_closed_loop_error_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
				return;
			}
			if (allowable_closed_loop_error != allowable_closed_loop_error_[index])
			{
				pidf_changed_[index] = true;
				allowable_closed_loop_error_[index] = allowable_closed_loop_error;
			}
		}
		int getAllowableClosedloopError(size_t index) const
		{
			if (index >= (sizeof(allowable_closed_loop_error_) / sizeof(allowable_closed_loop_error_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
				return 0;
			}
			return allowable_closed_loop_error_[index];
		}
		void setMaxIntegralAccumulator(int max_integral_accumulator, size_t index)
		{
			if (index >= (sizeof(max_integral_accumulator_) / sizeof(max_integral_accumulator_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
				return;
			}
			if (max_integral_accumulator != max_integral_accumulator_[index])
			{
				pidf_changed_[index] = true;
				max_integral_accumulator_[index] = max_integral_accumulator;
			}
		}
		int getMaxIntegralAccumulator(size_t index) const
		{
			if (index >= (sizeof(max_integral_accumulator_) / sizeof(max_integral_accumulator_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
				return 0.0;
			}
			return max_integral_accumulator_[index];
		}
		void setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index)
		{
			if (index >= (sizeof(closed_loop_peak_output_) / sizeof(closed_loop_peak_output_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setClosedLoopPeakOutput()");
				return;
			}
			if (closed_loop_peak_output != closed_loop_peak_output_[index])
			{
				pidf_changed_[index] = true;
				closed_loop_peak_output_[index] = closed_loop_peak_output;
			}
		}
		double getClosedLoopPeakOutput(size_t index) const
		{
			if (index >= (sizeof(closed_loop_peak_output_) / sizeof(closed_loop_peak_output_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getClosedLoopPeakOutput()");
				return 0.0;
			}
			return closed_loop_peak_output_[index];
		}

		void setClosedLoopPeriod(int closed_loop_period, size_t index)
		{
			if (index >= (sizeof(closed_loop_period_) / sizeof(closed_loop_period_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::setClosedLoopPeriod()");
				return;
			}
			if (closed_loop_period != closed_loop_period_[index])
			{
				pidf_changed_[index] = true;
				closed_loop_period_[index] = closed_loop_period;
			}
		}
		int getClosedLoopPeriod(size_t index) const
		{
			if (index >= (sizeof(closed_loop_period_) / sizeof(closed_loop_period_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getClosedLoopPeriod()");
				return 0.0;
			}
			return closed_loop_period_[index];
		}

		void setAuxPidPolarity(bool aux_pid_polarity)
		{
			if (aux_pid_polarity_ != aux_pid_polarity)
			{
				aux_pid_polarity_ = aux_pid_polarity;
				aux_pid_polarity_changed_ = true;
			}
		}
		bool getAuxPidPolarity(void) const
		{
			return aux_pid_polarity_;
		}

		void setIntegralAccumulator(double iaccum)
		{
			iaccum_ = iaccum;
			iaccum_changed_ = true;
		}
		double getIntegralAccumulator(void) const
		{
			return iaccum_;
		}
		bool integralAccumulatorChanged(double &iaccum)
		{
			iaccum = iaccum_;
			if (!iaccum_changed_)
				return false;
			iaccum_changed_ = false;
			return true;
		}

		void set(double command)
		{
			command_changed_ = command_ != command;
			command_ = command;
		}
		void setMode(TalonMode mode)
		{
			if ((mode <= TalonMode_Uninitialized) || (mode >= TalonMode_Last))
			{
				ROS_WARN("Invalid mode passed to TalonHWCommand::setMode()");
				return;
			}
			if (mode != mode_)
			{
				mode_         = mode;
				mode_changed_ = true;
			}
		}

		void setDemand1Type(DemandType demand_type)
		{
			if ((demand_type <  DemandType_Neutral) ||
				(demand_type >= DemandType_Last))
			{
				ROS_WARN("Invalid mode passed to TalonHWCommand::setDemand1Type()");
				return;
			}
			if (demand1_type_ != demand_type)
			{
				demand1_type_    = demand_type;
				demand1_changed_ = true;
			}
		}
		DemandType getDemand1Type(void) const
		{
			return demand1_type_;
		}

		void setDemand1Value(double demand_value)
		{
			if (demand1_value_ != demand_value)
			{
				demand1_value_   = demand_value;
				demand1_changed_ = true;
			}
		}
		double getDemand1Value(void) const
		{
			return demand1_value_;
		}
		bool demand1Changed(DemandType &type, double &value)
		{
			type  = demand1_type_;
			value = demand1_value_;
			if (!demand1_changed_)
				return false;
			demand1_changed_ = false;
			return true;
		}

		void setNeutralMode(NeutralMode neutral_mode)
		{
			if (neutral_mode == NeutralMode_Uninitialized)
				return; // Don't warn on this?
			else if ((neutral_mode < NeutralMode_Uninitialized) ||
					 (neutral_mode >= NeutralMode_Last))
			{
				ROS_WARN("Invalid neutral_mode passed to TalonHWCommand::setNeutralMode()");
				return;
			}
			if (neutral_mode != neutral_mode_)
			{
				neutral_mode_         = neutral_mode;
				neutral_mode_changed_ = true;
			}
		}
		bool getNeutralMode(void)
		{
			return neutral_mode_;
		}

		void setNeutralOutput(void)
		{
			neutral_output_ = true;
		}

		bool slotChanged(int &newpidfSlot)
		{
			newpidfSlot = pidf_slot_;
			if (!pidf_slot_changed_)
				return false;
			pidf_slot_changed_ = false;
			return true;
		}
		bool pidfChanged(double &p, double &i, double &d, double &f, int &iz, int &allowable_closed_loop_error, double &max_integral_accumulator, double &closed_loop_peak_output, int &closed_loop_period, size_t index)
		{
			if (index >= (sizeof(p_) / sizeof(p_[0])))
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::pidfChanged()");
				return false;
			}
			p = p_[index];
			i = i_[index];
			d = d_[index];
			f = f_[index];
			iz = i_zone_[index];
			allowable_closed_loop_error = allowable_closed_loop_error_[index];
			max_integral_accumulator = max_integral_accumulator_[index];
			closed_loop_peak_output = closed_loop_peak_output_[index];
			closed_loop_period = closed_loop_period_[index];
			if (!pidf_changed_[index])
				return false;
			pidf_changed_[index] = false;
			return true;
		}

		bool auxPidPolarityChanged(bool &aux_pid_polarity)
		{
			aux_pid_polarity = aux_pid_polarity_;
			if (!aux_pid_polarity_changed_)
				return false;
			aux_pid_polarity_changed_ = false;
			return true;
		}

		// Check to see if mode changed since last call
		// If so, return true and set mode to new desired
		// talon mode
		// If mode hasn't changed, return false
		// Goal here is to prevent writes to the CAN
		// bus to repeatedly set the mode to the same value.
		// Instead, only send a setMode to a given Talon if
		// the mode has actually changed.
		bool newMode(TalonMode &mode)
		{
			mode = mode_;
			if (!mode_changed_)
				return false;
			mode_changed_ = false;
			return true;
		}

		void setPidfSlot(int pidf_slot)
		{
			if (pidf_slot != pidf_slot_)
			{
				pidf_slot_ = pidf_slot;
				pidf_slot_changed_ = true;
			}
		}
		int getPidfSlot(void)const
		{
			return pidf_slot_;
		}

		void setInvert(bool invert)
		{
			if (invert != invert_)
			{
				invert_ = invert;
				invert_changed_ = true;
			}
		}
		void setSensorPhase(bool invert)
		{
			if (invert != sensor_phase_)
			{
				sensor_phase_ = invert;
				invert_changed_ = true;
			}
		}
		bool invertChanged(bool &invert, bool &sensor_phase)
		{
			invert = invert_;
			sensor_phase = sensor_phase_;
			if (!invert_changed_)
				return false;
			invert_changed_ = false;
			return true;
		}

		bool neutralModeChanged(NeutralMode &neutral_mode)
		{
			neutral_mode = neutral_mode_;
			if (!neutral_mode_changed_)
				return false;
			neutral_mode_changed_ = false;
			return true;
		}

		//output shaping
		bool outputShapingChanged(double &closed_loop_ramp,
								  double &open_loop_ramp,
								  double &peak_output_forward,
								  double &peak_output_reverse,
								  double &nominal_output_forward,
								  double &nominal_output_reverse,
								  double &neutral_deadband)
		{
			closed_loop_ramp = closed_loop_ramp_;
			open_loop_ramp = open_loop_ramp_;
			peak_output_forward = peak_output_forward_;
			peak_output_reverse = peak_output_reverse_;
			nominal_output_forward = nominal_output_forward_;
			nominal_output_reverse = nominal_output_reverse_;
			neutral_deadband = neutral_deadband_;
			if (!output_shaping_changed_)
				return false;
			output_shaping_changed_ = false;
			return true;
		}

		// Set motor controller to neutral output
		// This should be a one-shot ... only
		// write it to the motor controller once
		bool neutralOutputChanged(void)
		{
			if (!neutral_output_)
				return false;
			neutral_output_ = false;
			return true;
		}

		FeedbackDevice getEncoderFeedback(void) const
		{
			return encoder_feedback_;
		}
		void setEncoderFeedback(FeedbackDevice encoder_feedback)
		{
			if ((encoder_feedback >= FeedbackDevice_Uninitialized) &&
				(encoder_feedback <  FeedbackDevice_Last) )
			{
				if (encoder_feedback != encoder_feedback_)
				{
					encoder_feedback_ = encoder_feedback;
					encoder_feedback_changed_ = true;
				}
			}
			else
				ROS_WARN_STREAM("Invalid feedback device requested");
		}
		double getFeedbackCoefficient(void) const
		{
			return feedback_coefficient_;
		}
		void setFeedbackCoefficient(double feedback_coefficient)
		{
			if (feedback_coefficient != feedback_coefficient_)
			{
				feedback_coefficient_ = feedback_coefficient;
				encoder_feedback_changed_ = true;
			}
		}
		bool encoderFeedbackChanged(FeedbackDevice &encoder_feedback, double &feedback_coefficient)
		{
			encoder_feedback = encoder_feedback_;
			feedback_coefficient = feedback_coefficient_;
			if (!encoder_feedback_changed_)
				return false;
			encoder_feedback_changed_ = false;
			return true;
		}

		int getEncoderTicksPerRotation(void) const
		{
			return encoder_ticks_per_rotation_;
		}

		void setEncoderTicksPerRotation(int encoder_ticks_per_rotation)
		{
			encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
		}

		//output shaping
		void setClosedloopRamp(double closed_loop_ramp)
		{
			if (closed_loop_ramp_ != closed_loop_ramp)
			{
				closed_loop_ramp_ = closed_loop_ramp;
				output_shaping_changed_ = true;
			}
		}
		double getClosedloopRamp(void) const
		{
			return closed_loop_ramp_;
		}
		void setOpenloopRamp(double open_loop_ramp)
		{
			if (open_loop_ramp_ != open_loop_ramp)
			{
				open_loop_ramp_ = open_loop_ramp;
				output_shaping_changed_ = true;
			}
		}
		double getOpenloopRamp(void) const
		{
			return open_loop_ramp_;
		}

		void setPeakOutputForward(double peak_output_forward)
		{
			if (peak_output_forward != peak_output_forward_)
			{
				peak_output_forward_ = peak_output_forward;
				output_shaping_changed_ = true;
			}
		}
		double getPeakOutputForward(void) const
		{
			return peak_output_forward_;
		}

		void setPeakOutputReverse(double peak_output_reverse)
		{
			if (peak_output_reverse != peak_output_reverse_)
			{
				peak_output_reverse_ = peak_output_reverse;
				output_shaping_changed_ = true;
			}
		}
		double getPeakOutputReverse(void) const
		{
			return peak_output_reverse_;
		}

		void setNominalOutputForward(double nominal_output_forward)
		{
			if (nominal_output_forward != nominal_output_forward_)
			{
				nominal_output_forward_ = nominal_output_forward;
				output_shaping_changed_ = true;
			}
		}
		double getNominalOutputForward(void) const
		{
			return nominal_output_forward_;
		}

		void setNominalOutputReverse(double nominal_output_reverse)
		{
			if (nominal_output_reverse != nominal_output_reverse_)
			{
				nominal_output_reverse_ = nominal_output_reverse;
				output_shaping_changed_ = true;
			}
		}
		double getNominalOutputReverse(void) const
		{
			return nominal_output_reverse_;
		}

		void setNeutralDeadband(double neutral_deadband)
		{
			if (neutral_deadband != neutral_deadband_)
			{
				neutral_deadband_ = neutral_deadband;
				output_shaping_changed_ = true;
			}
		}
		double getNeutralDeadband(void) const
		{
			return neutral_deadband_;
		}

		void setVoltageCompensationSaturation(double voltage)
		{
			if (voltage != voltage_compensation_saturation_)
			{
				voltage_compensation_saturation_ = voltage;
				voltage_compensation_changed_ = true;
			}
		}
		double getVoltageCompensationSaturation(void) const
		{
			return voltage_compensation_saturation_;
		}

		void setVoltageMeasurementFilter(int filterWindowSamples)
		{
			if (filterWindowSamples != voltage_measurement_filter_)
			{
				voltage_measurement_filter_ = filterWindowSamples;
				voltage_compensation_changed_ = true;
			}
		}
		int getVoltageMeasurementFilter(void) const
		{
			return voltage_compensation_saturation_;
		}

		void setVoltageCompensationEnable(bool enable)
		{
			if (enable != voltage_compensation_enable_)
			{
				voltage_compensation_enable_ = enable;
				voltage_compensation_changed_ = true;
			}
		}

		bool getEnableVoltageCompenation(void) const
		{
			return voltage_compensation_enable_;
		}

		bool voltageCompensationChanged(double &voltage_compensation_saturation,
										int &voltage_measurement_filter,
										bool &voltage_compensation_enable)
		{
			voltage_compensation_saturation = voltage_compensation_saturation_;
			voltage_measurement_filter      = voltage_measurement_filter_;
			voltage_compensation_enable     = voltage_compensation_enable_;
			if (voltage_compensation_changed_)
			{
				voltage_compensation_changed_ = false;
				return true;
			}
			return false;
		}

		void setVelocityMeasurementPeriod(hardware_interface::VelocityMeasurementPeriod period)
		{
			if (period != velocity_measurement_period_)
			{
				velocity_measurement_period_ = period;
				velocity_measurement_changed_ = true;
			}
		}

		bool getVoltageMeasurementPeriod(void) const
		{
			return velocity_measurement_period_;
		}

		void setVelocityMeasurementWindow(int window)
		{
			if (window != velocity_measurement_window_)
			{
				velocity_measurement_window_ = window;
				velocity_measurement_changed_ = true;
			}
		}

		bool getVoltageMeasurementWindow(void) const
		{
			return velocity_measurement_window_;
		}

		bool velocityMeasurementChanged(hardware_interface::VelocityMeasurementPeriod &period,
										int &window)
		{
			period = velocity_measurement_period_;
			window = velocity_measurement_window_;
			if (velocity_measurement_changed_)
			{
				velocity_measurement_changed_ = false;
				return true;
			}
			return false;
		}

		void setSelectedSensorPosition(double position)
		{
			sensor_position_value_ = position;
			sensor_position_changed_ = true;
		}
		double getSelectedSensorPosition(void) const
		{
			return sensor_position_value_;
		}

		bool sensorPositionChanged(double &position)
		{
			position = sensor_position_value_;
			if (!sensor_position_changed_)
				return false;
			sensor_position_changed_ = false;
			return true;
		}

		void setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
		{
			if ((source != limit_switch_local_forward_source_) ||
					(normal != limit_switch_local_forward_normal_))
			{
				if ((source <= LimitSwitchSource_Uninitialized) ||
						(source >= LimitSwitchSource_Last))
				{
					ROS_WARN("Invalid source in setForwardLimitSwitchSource");
					return;
				}
				if ((normal <= LimitSwitchNormal_Uninitialized) ||
						(normal >= LimitSwitchNormal_Last))
				{
					ROS_WARN("Invalid normal in setForwardLimitSwitchSource");
					return;
				}
				if ((limit_switch_local_forward_source_ != source) ||
				    (limit_switch_local_forward_normal_ != normal) )
				{
					limit_switch_local_forward_source_ = source;
					limit_switch_local_forward_normal_ = normal;
					limit_switch_local_changed_ = true;
				}
			}
		}

		void getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_forward_source_;
			normal = limit_switch_local_forward_normal_;
		}

		void setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
		{
			if ((source != limit_switch_local_reverse_source_) || (normal != limit_switch_local_reverse_normal_))
			{
				if ((source <= LimitSwitchSource_Uninitialized) ||
						(source >= LimitSwitchSource_Last))
				{
					ROS_WARN("Invalid source in setReverseLimitSwitchSource");
					return;
				}
				if ((normal <= LimitSwitchNormal_Uninitialized) ||
						(normal >= LimitSwitchNormal_Last))
				{
					ROS_WARN("Invalid normal in setReverseLimitSwitchSource");
					return;
				}
				if ((limit_switch_local_reverse_source_ != source) ||
				    (limit_switch_local_reverse_normal_ != normal) )
				{
					limit_switch_local_reverse_source_ = source;
					limit_switch_local_reverse_normal_ = normal;
					limit_switch_local_changed_ = true;
				}
			}
		}

		void getReverseLimitSwitchSourceSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_reverse_source_;
			normal = limit_switch_local_reverse_normal_;
		}

		bool limitSwitchesSourceChanged(LimitSwitchSource &forward_source, LimitSwitchNormal &forward_normal, LimitSwitchSource &reverse_source, LimitSwitchNormal &reverse_normal)
		{
			forward_source = limit_switch_local_forward_source_;
			forward_normal = limit_switch_local_forward_normal_;
			reverse_source = limit_switch_local_reverse_source_;
			reverse_normal = limit_switch_local_reverse_normal_;
			if (!limit_switch_local_changed_)
				return false;
			limit_switch_local_changed_ = false;
			return true;
		}

		// softlimits
		void setForwardSoftLimitThreshold(double threshold)
		{
			if (threshold != softlimit_forward_threshold_)
			{
				softlimit_forward_threshold_ = threshold;
				softlimit_changed_ = true;
			}
		}
		double getForwardSoftLimitThreshold(void) const
		{
			return softlimit_forward_threshold_;
		}

		void setForwardSoftLimitEnable(bool enable)
		{
			if (enable != softlimit_forward_enable_)
			{
				softlimit_forward_enable_ = enable;
				softlimit_changed_ = true;
			}
		}
		bool getForwardSoftLimitEnable(void) const
		{
			return softlimit_forward_enable_;
		}
		void setReverseSoftLimitThreshold(double threshold)
		{
			if (threshold != softlimit_reverse_threshold_)
			{
				softlimit_reverse_threshold_ = threshold;
				softlimit_changed_ = true;
			}
		}
		double getReverseSoftLimitThreshold(void) const
		{
			return softlimit_reverse_threshold_;
		}

		void setReverseSoftLimitEnable(bool enable)
		{
			if (enable != softlimit_reverse_enable_)
			{
				softlimit_reverse_enable_ = enable;
				softlimit_changed_ = true;
			}
		}
		bool getReverseSoftLimitEnable(void) const
		{
			return softlimit_reverse_enable_;
		}

		void setOverrideSoftLimitsEnable(bool enable)
		{
			if (enable != softlimits_override_enable_)
			{
				softlimits_override_enable_ = enable;
				softlimit_changed_ = true;
			}
		}
		bool getOverrideSoftsLimitEnable(void) const
		{
			return softlimits_override_enable_;
		}

		bool SoftLimitChanged(double &forward_threshold, bool &forward_enable, double &reverse_threshold, bool &reverse_enable, bool &override_enable)
		{
			forward_threshold = softlimit_forward_threshold_;
			forward_enable = softlimit_forward_enable_;
			reverse_threshold = softlimit_reverse_threshold_;
			reverse_enable = softlimit_reverse_enable_;
			override_enable = softlimits_override_enable_;
			if (!softlimit_changed_)
				return false;
			softlimit_changed_ = false;
			return true;
		}

		// current limits
		void setPeakCurrentLimit(int amps)
		{
			if (amps != current_limit_peak_amps_)
			{
				current_limit_peak_amps_ = amps;
				current_limit_changed_ = true;
			}
		}
		int getPeakCurrentLimit(void) const
		{
			return current_limit_peak_amps_;
		}

		void setPeakCurrentDuration(int msec)
		{
			if (msec != current_limit_peak_msec_)
			{
				current_limit_peak_msec_ = msec;
				current_limit_changed_ = true;
			}
		}
		int getPeakCurrentDuration(void) const
		{
			return current_limit_peak_msec_;
		}
		void setContinuousCurrentLimit(int amps)
		{
			if (amps != current_limit_continuous_amps_)
			{
				current_limit_continuous_amps_ = amps;
				current_limit_changed_ = true;
			}
		}
		int getContinuousCurrentLimit(void) const
		{
			return current_limit_continuous_amps_;
		}
		void setCurrentLimitEnable(bool enable)
		{
			if (enable != current_limit_enable_)
			{
				current_limit_enable_ = enable;
				current_limit_changed_ = true;
			}
		}
		bool getCurrentLimitEnable(void) const
		{
			return current_limit_enable_;
		}

		bool currentLimitChanged(int &peak_amps, int &peak_msec, int &continuous_amps, bool &enable)
		{
			peak_amps = current_limit_peak_amps_;
			peak_msec = current_limit_peak_msec_;
			continuous_amps = current_limit_continuous_amps_;
			enable = current_limit_enable_;
			if (!current_limit_changed_)
				return false;
			current_limit_changed_ = false;
			return true;
		}

		void setMotionCruiseVelocity(double velocity)
		{
			if (velocity != motion_cruise_velocity_)
			{
				motion_cruise_velocity_ = velocity;
				motion_cruise_changed_ = true;
			}
		}
		double getMotionCruiseVelocity(void) const
		{
			return motion_cruise_velocity_;
		}
		void setMotionAcceleration(double acceleration)
		{
			if (acceleration != motion_acceleration_)
			{
				motion_acceleration_ = acceleration;
				motion_cruise_changed_ = true;
			}
		}
		double getMotionAcceleration(void) const
		{
			return motion_acceleration_;
		}

		bool motionCruiseChanged(double &velocity, double &acceleration)
		{
			velocity = motion_cruise_velocity_;
			acceleration = motion_acceleration_;
			if (!motion_cruise_changed_)
				return false;
			motion_cruise_changed_ = false;
			return true;
		}

		// This is a one shot - when set, it needs to
		// call the appropriate Talon function once
		// then clear itself
		void setClearMotionProfileTrajectories(void)
		{
			motion_profile_clear_trajectories_ = true;
		}
		bool getClearMotionProfileTrajectories(void) const
		{
			return motion_profile_clear_trajectories_;
		}
		bool clearMotionProfileTrajectoriesChanged(void)
		{
			if (!motion_profile_clear_trajectories_)
				return false;
			motion_profile_clear_trajectories_ = false;
			return true;
		}
		void PushMotionProfileTrajectory(const TrajectoryPoint &traj_pt)
		{
			motion_profile_trajectory_points_.push_back(traj_pt);
		}
		std::vector<TrajectoryPoint> getMotionProfileTrajectories(void) const
		{
			return motion_profile_trajectory_points_;
		}
		bool motionProfileTrajectoriesChanged(std::vector<TrajectoryPoint> &points)
		{
			if (motion_profile_trajectory_points_.size() != 0)
			{
				//ROS_WARN_STREAM("motionProfileTraectoriesChanged, mptp.size()=" << motion_profile_trajectory_points_.size());
				// Return up to 20 points at a time - too
				// many really slows down the hardware interface
				auto start = motion_profile_trajectory_points_.begin();
				auto end   = start + std::min((size_t)motion_profile_trajectory_points_.size(), (size_t)4000); //Intentionally very large
				points = std::vector<TrajectoryPoint>(start, end);
				motion_profile_trajectory_points_.erase(start, end);
				//ROS_WARN_STREAM("  returning points.size()=" << points.size());
				return true;
			}
			return false;
		}

		// This is a one shot - when set, it needs to
		// call the appropriate Talon function once
		// then clear itself
		void setClearMotionProfileHasUnderrun(void)
		{
			motion_profile_clear_has_underrun_ = true;
		}
		bool getClearMotionProfileHasUnderrun(void) const
		{
			return motion_profile_clear_has_underrun_;
		}
		bool clearMotionProfileHasUnderrunChanged(void)
		{
			if (!motion_profile_clear_has_underrun_)
				return false;
			motion_profile_clear_has_underrun_ = false;
			return true;
		}

		void setMotionControlFramePeriod(int msec)
		{
			if (msec != motion_profile_control_frame_period_)
			{
				motion_profile_control_frame_period_ = msec;
				motion_profile_control_frame_period_changed_ = true;
			}
		}
		int getMotionControlFramePeriod(void) const
		{
			return motion_profile_control_frame_period_;
		}
		bool motionControlFramePeriodChanged(int &msec)
		{
			msec = motion_profile_control_frame_period_;
			if (!motion_profile_control_frame_period_changed_)
				return false;
			motion_profile_control_frame_period_changed_ = false;
			return true;
		}

		void setMotionProfileTrajectoryPeriod(int msec)
		{
			if (msec != motion_profile_profile_trajectory_period_)
			{
				motion_profile_profile_trajectory_period_ = msec;
				motion_profile_profile_trajectory_period_changed_ = true;
			}
		}
		int getMotionProfileTrajectoryPeriod(void) const
		{
			return motion_profile_profile_trajectory_period_;
		}
		bool motionProfileTrajectoryPeriodChanged(int &msec)
		{
			msec = motion_profile_profile_trajectory_period_;
			if (!motion_profile_profile_trajectory_period_changed_)
				return false;
			motion_profile_profile_trajectory_period_changed_ = false;
			return true;
		}

		void setClearStickyFaults(void)
		{
			clear_sticky_faults_ = true;
		}
		bool getClearStickyFaults(void) const
		{
			return clear_sticky_faults_;
		}
		bool clearStickyFaultsChanged(void)
		{
			if (!clear_sticky_faults_)
				return false;
			clear_sticky_faults_ = false;
			return true;
		}

		void setConversionFactor(double conversion_factor)
		{
			if (conversion_factor != conversion_factor_)
			{
				conversion_factor_ = conversion_factor;
				conversion_factor_changed_ = true;
			}
		}
		double getConversionFactor(void) const
		{
			return conversion_factor_;
		}
		bool conversionFactorChanged(double &conversion_factor)
		{
			conversion_factor = conversion_factor_;
			if (!conversion_factor_changed_)
				return false;
			conversion_factor_changed_ = false;
			return true;
		}
		std::vector<int> getCustomProfileNextSlot(void) const
		{
			return custom_profile_next_slot_;
		}	
		void setCustomProfileNextSlot(const std::vector<int> &next_slot)
		{
			custom_profile_next_slot_ = next_slot;
		}
		double getCustomProfileHz(void) const
		{
			return custom_profile_hz_;
		}
		void setCustomProfileHz(const double &hz)
		{
			custom_profile_hz_ = hz;
		}
		void setCustomProfileRun(const bool &run)
		{
			custom_profile_run_ = run;
		}
		bool getCustomProfileRun(void) 
		{
			return custom_profile_run_;
		}
		void setCustomProfileSlot(const int &slot)
		{
			custom_profile_slot_ = slot;
		}
		int getCustomProfileSlot(void) const
		{
			return custom_profile_slot_;
		}
		void pushCustomProfilePoint(const CustomProfilePoint &point, int slot)
		{
			custom_profile_points_[slot].push_back(point);
			if(custom_profile_points_[slot].size() != 0)
			{
				custom_profile_total_time_[slot].push_back(custom_profile_total_time_[slot].back() + point.duration);
			}
			else
			{
				custom_profile_total_time_[slot].push_back(point.duration);
			}
		} 
		void pushCustomProfilePoints(const std::vector<CustomProfilePoint> &points, int slot)
		{
			int prev_size = custom_profile_points_.size();
			custom_profile_points_[slot].insert(custom_profile_points_[slot].end(), points.begin(), points.end());
			for(; prev_size <custom_profile_points_.size(); prev_size++)
			{
				if(prev_size != 0)
				{
					custom_profile_total_time_[slot].push_back(points[prev_size].duration + custom_profile_total_time_[slot][prev_size - 1]);
				}
				else
				{
					custom_profile_total_time_[slot].push_back(points[prev_size].duration);	
				}
			}	
		}
		void overwriteCustomProfilePoints(const std::vector<CustomProfilePoint> &points, int slot)
		{
			
			custom_profile_points_[slot] = points;
			custom_profile_total_time_[slot].resize(points.size());
			
			for(size_t i = 0; i < points.size(); i++)
			{

				if(i != 0)
				{
					custom_profile_total_time_[slot][i] = points[i].duration + custom_profile_total_time_[slot][i-1];
				}
				else
				{
					custom_profile_total_time_[slot][i] = points[i].duration;
				}
			}	
		}
		std::vector<CustomProfilePoint> getCustomProfilePoints(int slot) /*const*/ //TODO, can be const?
		{
			

			return custom_profile_points_[slot];
		}
		std::vector<double> getCustomProfileTime(int slot)  /*const*/ //TODO, can be const?
		{
			return custom_profile_total_time_[slot];
		}
		int getCustomProfileCount(int slot)  /*const*/ //TODO, can be const?
		{
			return custom_profile_points_[slot].size();
		}

	private:
		double    command_; // motor setpoint - % vbus, velocity, position, etc
		bool      command_changed_;
		TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
		bool      mode_changed_; // set if mode needs to be updated on the talon hw
		DemandType demand1_type_;
		double    demand1_value_;
		bool      demand1_changed_;
		//RG: shouldn't there be a variable for the peak voltage limits?
		int       pidf_slot_; // index 0 or 1 of the active PIDF slot
		bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon
		double    iaccum_;
		bool      iaccum_changed_;

		bool      invert_;
		bool      sensor_phase_;
		bool      invert_changed_;

		NeutralMode neutral_mode_;
		bool        neutral_mode_changed_;
		bool        neutral_output_;

		FeedbackDevice encoder_feedback_;
		double feedback_coefficient_;
		bool encoder_feedback_changed_;
		int encoder_ticks_per_rotation_;

		//output shaping
		double closed_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;
		bool   output_shaping_changed_;

		double voltage_compensation_saturation_;
		int   voltage_measurement_filter_;
		bool  voltage_compensation_enable_;
		bool  voltage_compensation_changed_;

		hardware_interface::VelocityMeasurementPeriod velocity_measurement_period_;
		int velocity_measurement_window_;
		bool velocity_measurement_changed_;

		double sensor_position_value_;
		bool sensor_position_changed_;

		LimitSwitchSource limit_switch_local_forward_source_;
		LimitSwitchNormal limit_switch_local_forward_normal_;
		LimitSwitchSource limit_switch_local_reverse_source_;
		LimitSwitchNormal limit_switch_local_reverse_normal_;
		bool limit_switch_local_changed_;
		RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		LimitSwitchNormal limit_switch_remote_forward_normal_;
		RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		LimitSwitchNormal limit_switch_remote_reverse_normal_;
		bool limit_switch_remote_changed_;

		double softlimit_forward_threshold_;
		bool softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool softlimit_reverse_enable_;
		bool softlimits_override_enable_;
		bool softlimit_changed_;

		int current_limit_peak_amps_;
		int current_limit_peak_msec_;
		int current_limit_continuous_amps_;
		bool current_limit_enable_;
		bool current_limit_changed_;

		// Talon expects these in integral sensorUnitsPer100ms,
		// but at this level we're still dealing with
		// radians/sec (or /sec^2 for acceleration)
		double motion_cruise_velocity_;
		double motion_acceleration_;
		bool motion_cruise_changed_;

		bool motion_profile_clear_trajectories_;
		bool motion_profile_clear_has_underrun_;
		std::vector<TrajectoryPoint> motion_profile_trajectory_points_;
		int motion_profile_control_frame_period_;
		bool motion_profile_control_frame_period_changed_;
		int motion_profile_profile_trajectory_period_;
		bool motion_profile_profile_trajectory_period_changed_;

		bool clear_sticky_faults_;

		// 2 entries in the Talon HW for each of these settings
		double p_[2];
		double i_[2];
		double d_[2];
		double f_[2];
		int    i_zone_[2];
		int    allowable_closed_loop_error_[2];
		double max_integral_accumulator_[2];
		double closed_loop_peak_output_[2];
		int    closed_loop_period_[2];
		bool   pidf_changed_[2];
		bool   aux_pid_polarity_;
		bool   aux_pid_polarity_changed_;

		double conversion_factor_;
		bool   conversion_factor_changed_;
		
		bool custom_profile_run_;
		int custom_profile_slot_;
		std::vector<int> custom_profile_next_slot_;
		double custom_profile_hz_;
		std::vector<std::vector<CustomProfilePoint>> custom_profile_points_;
		std::vector<std::vector<double>> custom_profile_total_time_;
		//TODO: error catching on slot



			

};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a Talon
class TalonCommandHandle: public TalonStateHandle
{
	public:
		TalonCommandHandle(void) :
			TalonStateHandle(),
			cmd_(0)
		{
		}

		TalonCommandHandle(const TalonStateHandle &js, TalonHWCommand *cmd) :
			TalonStateHandle(js),
			cmd_(cmd)
		{
			if (!cmd_)
				throw HardwareInterfaceException("Cannot create Talon handle '" + js.getName() + "'. command pointer is null.");
		}

		// Operator which allows access to methods from
		// the TalonHWCommand member var associated with this
		// handle
		// Note that we could create separate methods in
		// the handle class for every method in the HWState
		// class, e.g.
		//     double getFoo(void) const {assert(_state); return state_->getFoo();}
		// but if each of them just pass things unchanged between
		// the calling code and the HWState method there's no
		// harm in making a single method to do so rather than
		// dozens of getFoo() one-line methods
		//
		TalonHWCommand *operator->()
		{
			assert(cmd_);
			return cmd_;
		}

		// Get a pointer to the HW state associated with
		// this Talon.  Since CommandHandle is derived
		// from StateHandle, there's a state embedded
		// in each instance of a CommandHandle. Use
		// this method to access it.
		//
		// handle->state()->getCANID();
		//
		const TalonHWState *state(void) const
		{
			return TalonStateHandle::operator->();
		}

	private:
		TalonHWCommand *cmd_;
};

// Use ClaimResources here since we only want 1 controller
// to be able to access a given Talon at any particular time
class TalonCommandInterface : public HardwareResourceManager<TalonCommandHandle, ClaimResources> {};
}
