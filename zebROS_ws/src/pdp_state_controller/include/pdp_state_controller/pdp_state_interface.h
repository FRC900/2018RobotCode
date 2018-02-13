#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

//holds data from the PDP
class PDPHWState
{
	public:
		PDPHWState(void) :
			voltage_(0.0),
			temperature_(0.0),
			total_current_(0.0),
			total_power_(0.0),
			total_energy_(0.0),
			current_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
		{}

		//access and set
		double getVoltage(void) const 		{return voltage_;}
		double getTemperature(void) const 	{return temperature_;}
		double getTotalCurrent(void) const 	{return total_current_;}
		double getTotalPower(void) const 	{return total_power_;}
		double getTotalEnergy(void) const 	{return total_energy_;}
		double getCurrent(int channel) const {
			if(channel >= 0 && channel <= 15)
				return current_[channel]; 
			else
			{
				ROS_WARN_STREAM("Invalid channel. Cannot read current.");
				return 0.0;
			}
		}

		void setVoltage(double voltage) 			{voltage_ = voltage;}
		void setTemperature(double temperature)		{temperature_ = temperature;}
		void setTotalCurrent(double total_current)	{total_current_ = total_current;}
		void setTotalPower(double total_power)		{total_power_ = total_power;}
		void setTotalEnergy(double total_energy) 	{total_energy_ = total_energy;}
		void setCurrent(double current, int channel) {
			if(channel >= 0 && channel <= 15)
				current_[channel] = current;
			else
				ROS_WARN_STREAM("Invalid channel. Cannot set current.");
		}

	private:
		double voltage_;
		double temperature_;
		double total_current_;
		double total_power_;
		double total_energy_;
		double current_[16];

};

class PDPStateHandle
{
	public:
		PDPStateHandle(void) : state_(0) {}

		PDPStateHandle(const std::string &name, const PDPHWState *state) : 
			name_(name),
			state_(state)
		{
			if (!state)
				throw HardwareInterfaceException("Cannot create Talon state handle '" + name + "'. State pointer is null.");
		}
		std::string getName(void) const {return name_;}

		const PDPHWState *operator->() const
		{
			assert(state_);
			return state_;
		}

	private:
		std::string 	name_;
		const PDPHWState *state_;
};

class PDPStateInterface: public HardwareResourceManager<PDPStateHandle> {};
}
