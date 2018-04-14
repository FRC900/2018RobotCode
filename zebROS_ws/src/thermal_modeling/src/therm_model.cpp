#include <thermal_modeling/therm_model.h>

namespace thermal_model
{

	thermal_model::thermal_model(std::vector<node_properties> nodes, std::array<vector<double>, 2> efficiency_vs_rps, std::array<vector<double>, 2> air_speed_vs_rps, motor_properties properties)
	{
		nodes_ = nodes;
		properties_ =  properties;
		efficiency_curve_.set_points(efficiency_vs_rps[0], efficiency_vs_rps[1]);
		air_speed_curve_.set_points(air_speed_vs_rps[0], air_speed_vs_rps[1]);	
		heats_.resize(nodes_.size());
	}
	void thermal_model::iterate_model(const double &dt. const double &current, const double &voltage, const double &rps, std::vector<identified_val> assign_temp = {})
	{
		for(size_t i = 0; i < heats_.size(); i++)
		{
			heats_[i] = 0;
		}
		const double power = current * voltage;
		distribute_losses(dt, rps, power); 
		diffuse_heat(dt, rps);
		distribute_heat():
	}
	void thermal_model::distribute_losses(const double &dt, const double &rotation_rate, const double &power)
	{
		const double total_electrical_loss = dt * power * efficiency_curve_(rotation_rate) * properties_.proportion_losses_electrical;
		const double total_mechanical_loss = dt * power * efficiency_curve_(rotation_rate) * properties_.proportion_losses_mechanical;
		for(size_t i = 0; i < nodes_.size(); i++)
		{ 
			heats_[i] += nodes_[i].proportion_electrical_loss_absorb * total_electrical_loss; 
			heats_[i] += nodes_[i].proportion_mechanical_loss_absorb * total_mechanical_loss; 
		}
	}
	void thermal_model::diffuse_heat(const double &dt, const double &rotation_rate)
	{
		//much complexity very wow

	}
	void thermal_model::distribute_heat()
	{
		for(size_t i = 0; i < heats_.size(); i++)
		{
			nodes_[i].temperature += heats_[i] / nodes_[i].thermal_capacity;
		}	
	}
}
