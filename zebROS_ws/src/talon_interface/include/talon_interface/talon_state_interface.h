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
		TalonMode_PercentVbus,
		TalonMode_Position,      // CloseLoop
		TalonMode_Speed,         // CloseLoop
		TalonMode_Current,       // CloseLoop ?
		TalonMode_Voltage,
		TalonMode_Follower,
		TalonMode_MotionProfile,
		TalonMode_MotionMagic,
		TalonMode_Last
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
				pidf_p_ {0, 0},
				pidf_i_ {0, 0},
				pidf_d_ {0, 0},
				pidf_f_ {0, 0},
				pidf_izone_ {0, 0},
				closed_loop_error_(0),
				fwd_limit_switch_closed_(0),
				rev_limit_switch_closed_(0),
				talon_mode_(TalonMode_Uninitialized),
				v_compensation_ramp_rate_(0),
				can_id_(can_id),
				slot_(0),
				invert_(false),
				invert_sensor_direction_(false)
			{
			}

			double getSetpoint(void)      const {return setpoint_;}
			double getPosition(void)      const {return position_;}
			double getSpeed(void)         const {return speed_;}
			double getOutputVoltage(void) const {return output_voltage_;}
			int    getCANID(void)         const {return can_id_;}
			double getOutputCurrent(void) const {return output_current_;}
			double getBusVoltage(void)    const {return bus_voltage_;}
			double getPidfP(int index)    const {
			if((index == 0) || (index == 1))
				return pidf_p_[index];
			else {
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;}
			}
			double getPidfI(int index)    const {
			if((index == 0) || (index == 1))
				return pidf_i_[index];
			else {
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;}
			}
			double getPidfD(int index)    const {
			if((index == 0) || (index == 1))
				return pidf_d_[index];
			else {
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;}
			}
			double getPidfF(int index)    const {
			if((index == 0) || (index == 1))
				return pidf_f_[index];
			else {
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;}
			}
			double getPidfIzone(int index)     const {
			if((index == 0) || (index == 1))
				return pidf_izone_[index];
			else {
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");
				return 0.0;}
			}
			int getClosedLoopError(void)  const {return closed_loop_error_;}
			int getFwdLimitSwitch(void)   const {return fwd_limit_switch_closed_;}
			int getRevLimitSwitch(void)   const {return rev_limit_switch_closed_;}
			TalonMode getTalonMode(void)  const {return talon_mode_;}
			double getVCompensationRampRate(void) const {return v_compensation_ramp_rate_;}
			
			bool getInvert(void)          const {return invert_;}
			bool getInvertSensorDirection(void) const {return invert_sensor_direction_;}

			void setSetpoint(double setpoint)            {setpoint_ = setpoint;}
			void setPosition(double position)            {position_ = position;}
			void setSpeed(double speed)                  {speed_ = speed;}
			void setOutputVoltage(double output_voltage) {output_voltage_ = output_voltage;}
			void setOutputCurrent(double output_current) {output_current_ = output_current;}
			void setBusVoltage(double bus_voltage)       {bus_voltage_ = bus_voltage;}
			void setPidfP(double pidf_p, int index)	     {
			if((index == 0) || (index == 1))
				pidf_p_[index] = pidf_p;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");}
			void setPidfI(double pidf_i, int index)	     {
			if((index == 0) || (index == 1))
				pidf_i_[index] = pidf_i;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");}
			void setPidfD(double pidf_d, int index)	     {
			if((index == 0) || (index == 1))
				pidf_d_[index] = pidf_d;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");}
			void setPidfF(double pidf_f, int index)	     {
			if((index == 0) || (index == 1))
				pidf_f_[index] = pidf_f;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");}
			void setPidfIzone(double pidf_izone, int index)	     {
			if((index == 0) || (index == 1))
				pidf_izone_[index] = pidf_izone;
			else
				ROS_WARN_STREAM("Invalid index. Must be 0 or 1.");}
			void setClosedLoopError(int closed_loop_error) 	{closed_loop_error_ = closed_loop_error;}
			void setFwdLimitSwitch(int fwd_limit_switch_closed) {fwd_limit_switch_closed_ = fwd_limit_switch_closed;}
			void setRevLimitSwitch(int rev_limit_switch_closed) {rev_limit_switch_closed_ = rev_limit_switch_closed;}
			void setTalonMode(TalonMode talon_mode)	     {talon_mode_ = talon_mode;}
			void setVCompensationRampRate(double ramp_rate) {v_compensation_ramp_rate_ = ramp_rate;}
			void setSlot(int slot)      {slot_ = slot;}
			void setInvert(bool invert) {invert_ = invert;}
			void setInvertSensorDirection(bool invert) {invert_sensor_direction_ = invert;}


			// Add code to read and/or store all the other state from the Talon :
			// limit switch settings, sensing
			// pid slot selected and PIDF values
			// voltage compensation
			// voltage compensation stuff
			// etc, etc, etc
			//RG: I think there should be a set peak voltage function - that should go in talon_command since it is something sent to the talon. We could reflect that setting here, though
		private:
			double setpoint_;
			double position_;
			double speed_;
			double output_voltage_;
			double output_current_;
			double bus_voltage_;
			double pidf_p_[2];
			double pidf_i_[2];
			double pidf_d_[2];
			double pidf_f_[2];
			unsigned pidf_izone_[2];
			int closed_loop_error_;
			int fwd_limit_switch_closed_;
			int rev_limit_switch_closed_;
			TalonMode talon_mode_;
			double v_compensation_ramp_rate_; //voltage compensation

			int can_id_;

			int slot_;
			bool invert_;
			bool invert_sensor_direction_;
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
			std::string getName(void) const {return name_;}

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
			const TalonHWState * operator->() const {assert(state_); return state_;}

		private:
			std::string         name_;
			const TalonHWState *state_; // leave this const since state should never change the Talon itself
	};

	// Glue code to let this be registered in the list of
	// hardware resources on the robot.  Since state is
	// read-only, allow multiple controllers to register it.
	class TalonStateInterface : public HardwareResourceManager<TalonStateHandle> {};
}
