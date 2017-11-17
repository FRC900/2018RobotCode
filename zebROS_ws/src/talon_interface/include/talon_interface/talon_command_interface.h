#pragma once

#include <cassert>
#include <string>
#include <talon_interface/talon_state_interface.h>

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
		TalonMode_Current,       // CloseLoop
		TalonMode_Voltage,
		TalonMode_Follower,
		TalonMode_MotionProfile,
		TalonMode_MotionMagic,
		TalonMode_Last
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
			TalonHWCommand(void) :
				command_(0.),
				mode_(TalonMode_Uninitialized),
				mode_changed_(false),
				pidf_slot_(0),
				pidf_slot_changed_(false),
				pidf_changed_(false)
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
			// These get the requested setpoint, not the
			// status actually read from the controller
			// Need to think about which makes the most
			// sense to query...
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
			// Goal here is to prevent writes to the CAN
			// bus to repeatedly set the mode to the same value. 
			// Instead, only send a setMode to a given Talon if 
			// the mode has actually changed.
			bool newMode(TalonMode &mode)
			{
				if (!mode_changed_)
					return false;
				mode          = mode_;
				mode_changed_ = false;
				return true;
			}

		private:
			double    command_; // motor setpoint - % vbus, velocity, position, etc

			TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
			bool      mode_changed_; // set if mode needs to be updated on the talon hw

			int       pidf_slot_; // index 0 or 1 of the active PIDF slot
			bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon

			// 2 entries in the Talon HW for each of these settings
			double    p_[2];
			double    i_[2];
			double    i_zone_[2];
			double    d_[2];
			double    f_[2];
			bool      pidf_changed_;
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

			// Operator to call underlying methods from TalonHWCommand
			// object pointed to by this handle.
			TalonHWCommand * operator->() {assert(cmd_); return cmd_;}

		private:
			TalonHWCommand *cmd_;
	};

	// Use ClaimResources here since we only want 1 controller
	// to be able to access a given Talon at any particular time
	class TalonCommandInterface : public HardwareResourceManager<TalonCommandHandle, ClaimResources> {};
}
