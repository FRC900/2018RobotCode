#pragma once

#include <cassert>
#include <string>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{
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
			TalonHWState(void) :
				position_(0),
				speed_(0),
				output_voltage_(0)
			{}

			double getPosition(void)      const {return position_;}
			double getSpeed(void)         const {return speed_;}
			double getOutputVoltage(void) const {return output_voltage_;}

			void setPosition(double position)            {position_ = position;}
			void setSpeed(double speed)                  {speed_ = speed;}
			void setOutputVoltage(double output_voltage) {output_voltage_ = output_voltage;}

			const double *getPositionPtr(void) const { return &position_; }
			const double *getSpeedPtr   (void) const { return &speed_; }
			const double *getEffortPtr  (void) const { return &output_voltage_; }
			
			// Add code to read all the other state from the Talon :
			// output mode
			// limit switch settings, sensing
			// pid slot selected and PIDF values
			// voltage compensatino stuff
			// etc, etc, etc
		private:
			double position_;
			double speed_;
			double output_voltage_;
	};

	// Handle - used by each controller to get, by name of the
	// corresponding joint, an interface with which to get state
	// info about a Talon
	class TalonStateHandle : public JointStateHandle
	{
		public:
			TalonStateHandle(void) : 
				state_(0) 
			{}

			TalonStateHandle(const std::string &name, const TalonHWState *state) :
				JointStateHandle(name, state->getPositionPtr(), state->getSpeedPtr(), state->getEffortPtr()),
				state_(state)
			{
				if (!state)
					throw HardwareInterfaceException("Cannot create Talon state handle '" + name + "'. state pointer is null.");
			}

		private:
			const TalonHWState *state_; // leave this const since state should never change the Talon itself
	};

	// Glue code to let this be registered in the list of
	// hardware resources on the robot.  Since state is
	// read-only, allow multiple controllers to register it.
	class TalonStateInterface : public HardwareResourceManager<TalonStateHandle> {};

	// Sync these with values in CanTalonSRX
	enum TalonMode
	{
		TalonMode_Uninitialized = -1,
		TalonMode_PercentVbus,
		TalonMode_PositionCloseLoop,
		TalonMode_VelocityCloseLoop,
		TalonMode_CurrentCloseLoop,
		TalonMode_VoltCompen,
		TalonMode_SlaveFollower,
		TalonMode_MotionProfile,
		TalonMode_Last
	};
	// Class to buffer data needed to set the state of the
	// Talon.  This should (eventually) include anything
	// which might be set during runtime.  Config data
	// which is set only once at startup can be handled
	// in the hardware manager init rather than through
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
				mode_(TalonMode_Uninitialized),
				mode_changed_(false),
				pidf_slot_(0),
				pidf_slot_changed_(false)
			{
				for (int slot = 0; slot < 2; slot++)
				{
					p_[slot] =  0.0;
					i_[slot] =  0.0;
					d_[slot] =  0.0;
					f_[slot] =  0.0;
					i_zone_[slot] = 0.0;
				}
			}
			double get(void) const {return command_;}
			TalonMode getMode(void) const {return mode_;}

			void set(double command) {command_ = command;}
			void setMode(const TalonMode mode)
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
			// Check to see if mode changed since last call
			// If so, return true and set mode to new desired
			// talon mode
			// If mode hasn't changed, return false
			bool newMode(TalonMode &mode)
			{
				if (!mode_changed_)
					return false;
				mode          = mode_;
				mode_changed_ = false;
				return true;
			}

		private:
			double    command_;

			TalonMode mode_;
			bool      mode_changed_;

			int       pidf_slot_; // index 0 or 1 of the active PIDF slot
			bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon

			// 2 entries in the Talon HW for each of these settings
			double    p_[2];
			double    i_[2];
			double    i_zone_[2];
			double    d_[2];
			double    f_[2];
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

			void setCommand(double command) {assert(cmd_); cmd_->set(command);}
			double getCommand(void) const {assert(cmd_); return cmd_->get();}

			void setMode(const TalonMode mode) {assert(cmd_); cmd_->setMode(mode);}
			TalonMode getMode(void) const {assert(cmd_); return cmd_->getMode();}
			// Check to see if mode changed since last call
			// If so, return true and set mode to new desired
			// talon mode
			// If mode hasn't changed, return false
			bool newMode(TalonMode& mode) {assert(cmd_); return cmd_->newMode(mode);}

		private:
			TalonHWCommand *cmd_;
	};

	class TalonCommandInterface : public HardwareResourceManager<TalonCommandHandle, ClaimResources> {};

	/*
	class EffortTalonInterface : public TalonCommandInterface {};

	class VelocityTalonInterface : public TalonCommandInterface {};

	class PositionTalonInterface : public TalonCommandInterface {};
	*/
}
