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
				invert_(false),
				invert_sensor_direction_(false),
				invert_changed_(false)
			{
				for (int slot = 0; slot < 2; slot++)
				{
					p_[slot] =  0.0;
					i_[slot] =  0.0;
					d_[slot] =  0.0;
					f_[slot] =  0.0;
					i_zone_[slot] = 0.0;
					pidf_changed_[slot] = true;
				}
			}
			// This gets the requested setpoint, not the
			// status actually read from the controller
			// Need to think about which makes the most
			// sense to query...
			bool get(double &command)
			{
				command = command_;
				if (!command_changed_)
					return false;
				command_changed_ = false;
				return true;
			}

			TalonMode getMode(void) const {return mode_;}

			void setP(double oldP, int index){
				pidf_changed_[index] = true;
				p_[index] = oldP;}
			double getP(int index) const {return p_[index];}

			void setI(double ii, int index){
				pidf_changed_[index] = true;
				i_[index] = ii;}
			double getI(int index) const {return i_[index];}
			
			void setPID(double oldP, double oldI, double oldD, int index){
				pidf_changed_[index] = true;
				p_[index] = oldP;i_[index] =oldI;d_[index]=oldD;}
			void setPID(double oldP, double oldI, double oldD, double oldF, int index){
				pidf_changed_[index] = true;
				p_[index]=oldP;i_[index]=oldI;d_[index]=oldD;f_[index]=oldF;}

			void setD(double dd, int index){
				pidf_changed_[index] = true;
				d_[index] = dd;}
			double getD(int index) const {return d_[index];}

			void setF(double ff, int index){
				pidf_changed_[index] = true;
				f_[index] = ff;}
			double getF(int index){return f_[index];}

			void setIZ(unsigned oldIZ, int index){
				pidf_changed_[index] = true;
				i_zone_[index] = oldIZ;}
			unsigned getIZ(int index) const {return i_zone_[index];}

			void set(double command) {command_changed_ = true; command_ = command;}
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

			bool slotChanged(int &newpidfSlot)
			{
				newpidfSlot = pidf_slot_;
				if (!pidf_slot_changed_)
					return false;
				pidf_slot_changed_ = false;
				return true;
			}
			bool pidfChanged(double &p, double &i, double &d, double &f, unsigned &iz, int index){
				p = p_[index];
				i = i_[index];
				d = d_[index];
				f = f_[index];
				iz = i_zone_[index];
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
			void setInvertSensorDirection(bool invert) {invert_sensor_direction_ = invert; invert_changed_ = true;}
			bool invertChanged(bool &invert, bool &invert_sensor_direction)
			{
				invert = invert_;
				invert_sensor_direction = invert_sensor_direction_;
				if (!invert_changed_)
					return false;
				invert_changed_ = false;
				return true;
			}

		private:
			double    command_; // motor setpoint - % vbus, velocity, position, etc
			bool      command_changed_;
			TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
			bool      mode_changed_; // set if mode needs to be updated on the talon hw
			double    ramprate;
			//RG: shouldn't there be a variable for the peak voltage limits?
			int       pidf_slot_; // index 0 or 1 of the active PIDF slot
			bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon

			// 2 entries in the Talon HW for each of these settings
			double    p_[2];
			double    i_[2];
			unsigned  i_zone_[2];
			double    d_[2];
			double    f_[2];
			bool      pidf_changed_[2];
			bool      invert_;
			bool      invert_sensor_direction_;
			bool      invert_changed_;
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
