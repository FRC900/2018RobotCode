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
					pidf_changed_[slot] = true;
				}
			}
			// These get the requested setpoint, not the
			// status actually read from the controller
			// Need to think about which makes the most
			// sense to query...
			double get(void) const {return command_;}

			TalonMode getMode(void) const {return mode_;}

			void setP(double oldP){
				pidf_changed_[pidf_slot_] = true;
				p_[pidf_slot_] = oldP;}
			double getP(void) const {return p_[pidf_slot_];}

			void setI(double ii){
				pidf_changed_[pidf_slot_] = true;
				i_[pidf_slot_] = ii;}
			double getI(void) const {return i_[pidf_slot_];}
			
			void setPID(double oldP, double oldI, double oldD){
				pidf_changed_[pidf_slot_] = true;
				p_[pidf_slot_] = oldP;i_[pidf_slot_] =oldI;d_[pidf_slot_]=oldD;}
			virtual void setPID(double oldP, double oldI, double oldD, double oldF){
				pidf_changed_[pidf_slot_] = true;
				p_[pidf_slot_]=oldP;i_[pidf_slot_]=oldI;d_[pidf_slot_]=oldD;f_[pidf_slot_]=oldF;}

			void setD(double dd){
				pidf_changed_[pidf_slot_] = true;
				d_[pidf_slot_] = dd;}
			double getD(void) const {return d_[pidf_slot_];}

			void setF(double ff){
				pidf_changed_[pidf_slot_] = true;
				f_[pidf_slot_] = ff;}
			double getF(void){return f_[pidf_slot_];}

			void setIZ(unsigned oldIZ){
				pidf_changed_[pidf_slot_] = true;
				i_zone_[pidf_slot_] = oldIZ;}
			unsigned getIZ(void) const {return i_zone_[pidf_slot_];}

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

			bool slotChanged(int &newpidfSlot)
			{
				newpidfSlot = pidf_slot_;
				if (!pidf_slot_changed_)
					return false;
				pidf_slot_changed_ = false;
				return true;
			}
			bool pidfChanged(double &p, double &i, double &d, double &f, unsigned &iz){
				p = p_[pidf_slot_];
				i = i_[pidf_slot_];
				d = d_[pidf_slot_];
				f = f_[pidf_slot_];
				iz = i_zone_[pidf_slot_];
				if (!pidf_changed_[pidf_slot_])
					return false;
				pidf_slot_changed_ = false;
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

		private:
			double    command_; // motor setpoint - % vbus, velocity, position, etc
			TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
			bool      mode_changed_; // set if mode needs to be updated on the talon hw
			double    ramprate;
			int       pidf_slot_; // index 0 or 1 of the active PIDF slot
			bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon

			// 2 entries in the Talon HW for each of these settings
			double    p_[2];
			double    i_[2];
			unsigned    i_zone_[2];
			double    d_[2];
			double    f_[2];
			bool      pidf_changed_[2];
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
