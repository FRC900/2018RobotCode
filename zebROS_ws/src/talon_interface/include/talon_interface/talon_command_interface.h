#pragma once

#include <cassert>
#include <string>
#include <talon_interface/talon_state_interface.h>

namespace hardware_interface
{
	
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
			TalonHWCommand(void) :
				command_(0.),
				command_changed_(false),
				mode_(TalonMode_Uninitialized),
				mode_changed_(false),
				pidf_slot_(0),
				pidf_slot_changed_(false),
				iaccum_(0.0),
				iaccum_changed_(false),
				invert_(false),
				sensor_phase_(false),
				invert_changed_(false),
				neutral_mode_(NeutralMode_Uninitialized),
				neutral_mode_changed_(false),
				neutral_output_(false)
			{
				for (int slot = 0; slot < 2; slot++)
				{
					p_[slot] = 0.0;
					i_[slot] = 0.0;
					d_[slot] = 0.0;
					f_[slot] = 0.0;
					i_zone_[slot] = 0;
					allowable_closed_loop_error_[slot] = 0;
					max_integral_accumulator_[slot] = 0;

					pidf_changed_[slot] = true;
				}
			}
			// This gets the requested setpoint, not the
			// status actually read from the controller
			// Need to think about which makes the most
			// sense to query...
			bool get(float &command)
			{
				command = command_;
				if (!command_changed_)
					return false;
				command_changed_ = false;
				return true;
			}

			TalonMode getMode(void) const {return mode_;}

			void setP(float oldP, int index){
				if ((index < 0) || (index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setP()");
					return;
				}
				pidf_changed_[index] = true;
				p_[index] = oldP;}
			float getP(int index) const {
				if ((index < 0) || (index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getP()");
					return 0.0;
				}
				return p_[index];
			}

			void setI(float ii, int index){
				if ((index < 0) || (index >= (sizeof(i_) / sizeof(i_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setI()");
					return;
				}
				pidf_changed_[index] = true;
				i_[index] = ii;}
			float getI(int index) const {
				if ((index < 0) || (index >= (sizeof(i_) / sizeof(i_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getI()");
					return 0.0;
				}
				return i_[index];
			}

			void setD(float dd, int index){
				if ((index < 0) || (index >= (sizeof(d_) / sizeof(d_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setD()");
					return;
				}
				pidf_changed_[index] = true;
				d_[index] = dd;}
			float getD(int index) const {
				if ((index < 0) || (index >= (sizeof(d_) / sizeof(d_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getD()");
					return 0.0;
				}
				return d_[index];
			}

			void setF(float ff, int index){
				if ((index < 0) || (index >= (sizeof(f_) / sizeof(f_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setF()");
					return;
				}
				pidf_changed_[index] = true;
				f_[index] = ff;}
			float getF(int index){
				if ((index < 0) || (index >= (sizeof(f_) / sizeof(f_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getF()");
					return 0.0;
				}
				return f_[index];
			}

			void setIZ(int oldIZ, int index){
				if ((index < 0) || (index >= (sizeof(i_zone_) / sizeof(i_zone_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setIZ()");
					return;
				}
				pidf_changed_[index] = true;
				i_zone_[index] = oldIZ;}
			int getIZ(int index) const {
				if ((index < 0) || (index >= (sizeof(i_zone_) / sizeof(i_zone_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getIZ()");
					return 0.0;
				}
				return i_zone_[index];
			}

			void setAllowableClosedloopError(int allowable_closed_loop_error, int index) {
				if ((index < 0) || (index >= (sizeof(allowable_closed_loop_error_) / sizeof(allowable_closed_loop_error_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
					return;
				}
				pidf_changed_[index] = true;
				allowable_closed_loop_error_[index] = allowable_closed_loop_error;
			}
			int getAllowableClosedloopError(int index) const {
				if ((index < 0) || (index >= (sizeof(allowable_closed_loop_error_) / sizeof(allowable_closed_loop_error_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
					return 0;
				}
				return allowable_closed_loop_error_[index];
			}
			void setMaxIntegralAccumulator(int max_integral_accumulator, int index) {
				if ((index < 0) || (index >= (sizeof(max_integral_accumulator_) / sizeof(max_integral_accumulator_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
					return;
				}
				pidf_changed_[index] = true;
				max_integral_accumulator_[index] = max_integral_accumulator;
			}
			int getMaxIntegralAccumulator(int index) const {
				if ((index < 0) || (index >= (sizeof(max_integral_accumulator_) / sizeof(max_integral_accumulator_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
					return 0.0;
				}
				return max_integral_accumulator_[index];
			}
			
			void setPID(float oldP, float oldI, float oldD, int index){
				if ((index < 0) || (index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setPID()");
					return;
				}
				pidf_changed_[index] = true;
				p_[index] = oldP;i_[index] =oldI;d_[index]=oldD;}
			void setPID(float oldP, float oldI, float oldD, float oldF, int index){
				if ((index < 0) || (index >= (sizeof(p_) / sizeof(p_[0]))))
				{
					ROS_WARN("Invalid index passed to TalonHWCommand::setPIF()");
					return;
				}
				pidf_changed_[index] = true;
				p_[index]=oldP;i_[index]=oldI;d_[index]=oldD;f_[index]=oldF;}

			void setIntegralAccumulator(float iaccum)
			{
				iaccum_ = iaccum;
				iaccum_changed_ = true;
			}
			float getIntegralAccumulator(void) const { return iaccum_; }
			bool integralAccumulatorChanged(float &iaccum)
			{
				iaccum = iaccum_;
				if (!iaccum_changed_)
					return false;
				iaccum_changed_ = true;
				return true;
			}

			void set(float command) {command_changed_ = true; command_ = command;}
			void setMode(TalonMode mode)
			{
				if ((mode <= TalonMode_Uninitialized) || (mode >= TalonMode_Last))
				{
					ROS_WARN("Invalid mode passed to TalonHWCommand::setMode()");
					return;
				}
				mode_         = mode;
				mode_changed_ = true;
				this->set(0); // ??? Clear out setpoint for old mode
			}

			void setNeutralMode(NeutralMode neutral_mode)
			{
				if ((neutral_mode <= NeutralMode_Uninitialized) || (neutral_mode >= NeutralMode_Last))
				{
					ROS_WARN("Invalid neutral_mode passed to TalonHWCommand::setNeutralMode()");
					return;
				}
				neutral_mode_         = neutral_mode;
				neutral_mode_changed_ = true;
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
			bool pidfChanged(float &p, float &i, float &d, float &f, int &iz, int allowable_closed_loop_error, float max_integral_accumulator, int index){
				if ((index < 0) || (index >= (sizeof(p_) / sizeof(p_[0]))))
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
				if (!pidf_changed_[index])
					return false;
				pidf_changed_[index] = false;
				return true;
			}

			// Check  to see if mode changed since last call
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

			void setPidfSlot(int npidf_slot){pidf_slot_ = npidf_slot;pidf_slot_changed_ = true;}
			int getPidfSlot(void)const{return pidf_slot_;}

			void setInvert(bool invert) {invert_ = invert; invert_changed_ = true;}
			void setSensorPhase(bool invert) {sensor_phase_ = invert; invert_changed_ = true;}
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

			//general
			void Disable(){ }
			void Enable(){ }
			void ClearStickyFaults(){ }

			//voltage
			void SetVoltageRampRate(double rampRate){ }
			virtual void SetVoltageCompensationRampRate(double rampRate){ } 
			void ConfigNeutralMode(NeutralMode mode){ }
			void ConfigPeakOutputVoltage(double forwardVoltage, double reverseVoltage){ }
			void SetNominalClosedLoopVoltage(double voltage){ }

			//closed loop control/sensors
			virtual void SetAnalogPosition(int newPosition){ }
			virtual void SetEncPosition(int){ }
			void SetNumberOfQuadIdxRises(int rises){ }
			virtual void SetPulseWidthPosition(int newpos){ }
			void ConfigEncoderCodesPerRev(uint16_t codesPerRev){ }
			void ConfigPotentiometerTurns(uint16_t turns) { }
			void ConfigSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition){ }
			void EnableZeroSensorPositionOnIndex(bool enable, bool risingEdge){ }
			void EnableZeroSensorPositiononForwardLimit(bool enable){ }
			void EnableZeroSensorPositionOnReverseLimit(bool enable){ }
			//void SetFeedbackDevice(FeedbackDevice device){ }
			void SetSensorDirection(bool reverseSensor){ }
			void SetClosedLoopOutputDirection(bool reverseOutput){ }
			void SetCloseLoopRampRate(double rampRate){ }

			//limits
			void ConfigForwardLimit(double forwardLimitPosition){ }
			void ConfigReverseLimit(double reverseLimitPosition){ }
			void ConfigLimitSwitchOverrides(bool bForwardLimitSwitchEn, bool bReverseLimitSwitchEn){ }
			void ConfigForwardSoftLimitEnable(bool bForwardSoftLimitEn){ }
			void ConfigReverseSoftLimitEnable(bool bReverseSoftLimitEn){ }
			void ConfigFwdLimitSwitchNormallyOpen(bool normallyOpen){ }
			void ConfigRevLimitSwitchNormallyOpen(bool normallyOpen){ }
			void ConfigLimitMode(int mode){ }

			//motion profiling
			void ChangeMotionControlFramePeriod(int period){ }
			void ClearMotionProfileTrajectories(){ }
			//bool PushMotionProfileTrajectory(const TrajectoryPoint& trajPt){ }
			void ProcessMotionProfileBuffer(){ }
			void ClearMotionProfileHasUnderrun(){ }
			int SetMotionMagicCruiseVelocity(double motMagicCruiseVeloc){ }
			int SetMotionMagicAcceleration(double motMagicAccel){ }
			int SetCurrentLimit(uint32_t amps){ }
			int EnableCurrentLimit(bool enable){ }
			int SetCustomParam0(int32_t value){ }
			int SetCustomParam1(int32_t value){ }
			//void SetInverted(bool isInverted) override{ }
			void SetDataPortOutputPeriod(uint32_t periodMs){ }
			void SetDataPortOutputEnable(uint32_t idx, bool enable){ }
			void SetDataPortOutput(uint32_t idx, uint32_t OnTimeMs){ }

		private:
			float     command_; // motor setpoint - % vbus, velocity, position, etc
			bool      command_changed_;
			TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
			bool      mode_changed_; // set if mode needs to be updated on the talon hw
			float     ramprate;
			//RG: shouldn't there be a variable for the peak voltage limits?
			int       pidf_slot_; // index 0 or 1 of the active PIDF slot
			bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon

			// 2 entries in the Talon HW for each of these settings
			float     p_[2];
			float     i_[2];
			int       i_zone_[2];
			float     d_[2];
			float     f_[2];
			int       allowable_closed_loop_error_[2];
			float     max_integral_accumulator_[2];
			bool      pidf_changed_[2];

			float     iaccum_;
			bool      iaccum_changed_;

			bool      invert_;
			bool      sensor_phase_;
			bool      invert_changed_;

			NeutralMode neutral_mode_;
			bool        neutral_mode_changed_;
			bool        neutral_output_;
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
			//     float getFoo(void) const {assert(_state); return state_->getFoo();}
			// but if each of them just pass things unchanged between
			// the calling code and the HWState method there's no
			// harm in making a single method to do so rather than
			// dozens of getFoo() one-line methods
			// 
			TalonHWCommand * operator->() {assert(cmd_); return cmd_;}

			// Get a pointer to the HW state associated with
			// this Talon.  Since CommandHandle is derived
			// from StateHandle, there's a state embedded
			// in each instance of a CommandHandle. Use
			// this method to access it.
			//
			// handle->state()->getCANID();
			//
			const TalonHWState * state(void) const { return TalonStateHandle::operator->(); }

		private:
			TalonHWCommand *cmd_;
	};

	// Use ClaimResources here since we only want 1 controller
	// to be able to access a given Talon at any particular time
	class TalonCommandInterface : public HardwareResourceManager<TalonCommandHandle, ClaimResources> {};
}
