#include <thermal_modeling/thermal_model.h>

namespace thermal_modeling
{


	thermal_model::thermal_model(std::map<std::string, node_properties> nodes, std::array<std::vector<double>, 2> efficiency_vs_rps, std::array<std::vector<double>, 2> air_speed_vs_rps, motor_properties properties):
		properties_(properties)
	{
		
		nodes_.resize(nodes.size());
		
		int i = 0;
		for(auto &node : nodes)
		{ 
			nodes_[i] = node.second;
			node_indexes_.insert(std::pair<std::string, int>(node.first, i);
		}
		for(size_t j = 0; j < nodes_.size(); j++)
		{
			for(size_t k = 0; k < nodes_[j].connections_emissive.size(); k++)
			{
				if(!nodes_[j].connections_emissive[k].infinite_thermal_sink)
					nodes_[j].connections_emissive[k].index = node_indexes[nodes_[j].connections_emissive.id];
			}
			for(size_t k = 0; k < nodes_[j].connections_conductive.size(); k++)
			{
				if(!nodes_[j].connections_conductive[k].infinite_thermal_sink)
					nodes_[j].connections_conductive[k].index = node_indexes[nodes_[j].connections_conductive.id];
			}
			for(size_t k = 0; k < nodes_[j].connections_convective.size(); k++)
			{
				if(!nodes_[j].connections_convective[k].infinite_thermal_sink)
					nodes_[j].connections_convective[k].index = node_indexes[nodes_[j].connections_convective.id];
			}
		}
		efficiency_curve_.set_points(efficiency_vs_rps[0], efficiency_vs_rps[1]);
		air_speed_curve_.set_points(air_speed_vs_rps[0], air_speed_vs_rps[1]);	
	

		//TODO: Consider normalization instead of below error checking
		if(fabs(properties_.proportion_losses_mechanical + properties_.proportion_losses_electrical - 1) > .02)
			throw "motor property loss proportions don't sum to 1 (greater than 2\% difference)";

		double sum_proportion_electrical_loss_absorb = 0;
		double sum_proportion_mechanical_loss_absorb = 0;
		for(auto &node : nodes_)
		{ 
			sum_proportion_electrical_loss_absorb += node.proportion_electrical_loss_absorb;
			sum_proportion_mechanical_loss_absorb += node.proportion_mechanical_loss_absorb;
		}
		if(fabs(sum_proportion_electrical_loss_absorb - 1) > .02)
			throw "total electrical loss proportions don't sum to 1 (greater than 2\% difference)";
		
		if(fabs(sum_proportion_mechanical_loss_absorb - 1) > .02)
			throw "total mechanical loss proportions don't sum to 1 (greater than 2\% difference)";	
	}
	void thermal_model::compute_coupled_ode_deriv(const ode_state_type &temps, ode_state_type &dtempdt, const double /* t */)
	{
		for(int i =  0; i < dtempdt.size(); i++)
		{	
			dtempdt[i] = 0;	
		}

		for(int i =  0; i < dtempdt.size(); i++)
		{ 
			for(size_t k = 0; k < node.connections.size(); k++)
			{
				switch(nodes_[i].connections[k].type)
				{
					case emissive:	
						emissive_deriv(nodes_[i], dtempdt);
						break;
					case conductive:
						conductive_deriv(nodes_[i], dtempdt);
						break;
					default:			
						throw "attempted to calc deriv for unknown connection type";
						break; //break here as alternative to above line (shouldn't do anything)
				}
			}
			i++;
		}
	}
	//Generae function by using a combination of for loops and switch statements

	void thermal_model::iterate_model(const double &dt, const double &current, const double &voltage, const double &rps, std::vector<identified_val> assign_temp)
	{
		// integrate_const( stepper , system , x0 , t0 , t1 , dt ) 

		for(size_t i = 0; i < heats_.size(); i++)
		{
			heats_[i] = 0;
		}
		const double power = current * voltage;
		distribute_losses(dt, rps, power); 
		diffuse_heat(dt, rps);
		distribute_heat();
	}
	void thermal_model::distribute_losses(const double &dt, const double &rotation_rate, const double &power)
	{
		const double total_electrical_loss = dt * power * efficiency_curve_(rotation_rate) * properties_.proportion_losses_electrical;
		const double total_mechanical_loss = dt * power * efficiency_curve_(rotation_rate) * properties_.proportion_losses_mechanical;
		int i = 0;
		for(auto &node : nodes_)
		{ 
			heats_[i] += node.second.proportion_electrical_loss_absorb * total_electrical_loss; 
			heats_[i] += node.second.proportion_mechanical_loss_absorb * total_mechanical_loss; 
			i++;
		}
	}
	void thermal_model::diffuse_heat(const double &dt, const double &rotation_rate)
	{
		//much complexity very wow

	}
	void thermal_model::distribute_heat()
	{
		int i = 0;
		for(auto &node : nodes_)
		{ 
			node.second.temperature += heats_[i] / node.second.thermal_capacity;
			i++;
		}	
	}
}
