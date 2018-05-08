#include <thermal_modeling/thermal_model.h>

namespace thermal_modeling
{


	thermal_model::thermal_model(std::map<std::string, node_properties> nodes, std::array<std::vector<double>, 2> efficiency_vs_rps, std::array<std::vector<double>, 2> air_speed_vs_rps, motor_properties properties, std::vector<double> initial_temperatures):
		properties_(properties),
		temperatures_(initial_temperatures)
	{
		
		for(auto &node : nodes)
		{ 
			nodes_.push_back(node.second);
			node_indexes_.insert(std::pair<std::string, int>(node.first, nodes_.size() - 1));
		}
		const_dtempdt_adder_.resize(nodes_.size());
		for(size_t j = 0; j < nodes_.size(); j++)
		{
			for(size_t k = 0; k < nodes_[j].connections_emissive.size(); k++)
			{
				if(!nodes_[j].connections_emissive[k].infinite_thermal_sink)
					nodes_[j].connections_emissive[k].index = node_indexes_[nodes_[j].connections_emissive[k].id];
			}
			for(size_t k = 0; k < nodes_[j].connections_conductive.size(); k++)
			{
				if(!nodes_[j].connections_conductive[k].infinite_thermal_sink)
					nodes_[j].connections_conductive[k].index = node_indexes_[nodes_[j].connections_conductive[k].id];
			}
			for(size_t k = 0; k < nodes_[j].connections_natural_convective.size(); k++)
			{
				if(!nodes_[j].connections_natural_convective[k].infinite_thermal_sink)
					nodes_[j].connections_natural_convective[k].index = node_indexes_[nodes_[j].connections_natural_convective[k].id];
			}
			for(size_t k = 0; k < nodes_[j].connections_fan_convective.size(); k++)
			{
				if(!nodes_[j].connections_fan_convective[k].infinite_thermal_sink)
					nodes_[j].connections_fan_convective[k].index = node_indexes_[nodes_[j].connections_fan_convective[k].id];
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
			dtempdt[i] = const_dtempdt_adder_[i];	
		}

		for(int i =  0; i < dtempdt.size(); i++)
		{ 
			for(size_t k = 0; k < nodes_[i].connections_emissive.size(); k++)
			{
				emissive_deriv(i, k, dtempdt, temps);
			}
			for(size_t k = 0; k < nodes_[i].connections_conductive.size(); k++)
			{
				conductive_deriv(i, k, dtempdt, temps);
			}
			for(size_t k = 0; k < nodes_[i].connections_natural_convective.size(); k++)
			{
				natural_convective_deriv(i, k, dtempdt, temps);
			}
			for(size_t k = 0; k < nodes_[i].connections_fan_convective.size(); k++)
			{
				fan_convective_deriv(i, k, dtempdt, temps);
			}
		}
	}
	
	void thermal_model::emissive_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps)
	{
		const double other_temp = nodes_[i].connections_emissive[k].infinite_thermal_sink ? 
		nodes_[i].connections_emissive[k].infinite_thermal_sink_temp : 
		temps[nodes_[i].connections_emissive[k].index];

		const double Q = nodes_[i].connections_emissive[k].h * (temps[i] * temps[i] * temps[i] * temps[i] - 
		other_temp * other_temp * other_temp * other_temp); 

		//does this equation even apply to the typical infinite sink type situation?

		dtempdt[i] -= Q / nodes_[i].thermal_capacity;
		if(!nodes_[i].connections_emissive[k].infinite_thermal_sink)
			dtempdt[nodes_[i].connections_emissive[k].index] += Q/ nodes_[nodes_[i].connections_emissive[k].index].thermal_capacity;
		//energy conserved for this operation unless infinite sink	
	}
	void thermal_model::conductive_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps)
	{

		const double other_temp = nodes_[i].connections_emissive[k].infinite_thermal_sink ? 
		nodes_[i].connections_emissive[k].infinite_thermal_sink_temp : 
		temps[nodes_[i].connections_emissive[k].index];
		
		const double Q = nodes_[i].connections_conductive[k].h * (temps[i] - other_temp);
	
		
		dtempdt[i] -= Q / nodes_[i].thermal_capacity;
		if(!nodes_[i].connections_emissive[k].infinite_thermal_sink)
			dtempdt[nodes_[i].connections_emissive[k].index] += Q/ nodes_[nodes_[i].connections_emissive[k].index].thermal_capacity;


	}
	void thermal_model::natural_convective_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps)
	{
		const double other_temp = nodes_[i].connections_emissive[k].infinite_thermal_sink ? 
		nodes_[i].connections_emissive[k].infinite_thermal_sink_temp : 
		temps[nodes_[i].connections_emissive[k].index];
		
		const double Q = nodes_[i].connections_conductive[k].h * (temps[i] - other_temp);
	
		
		dtempdt[i] -= Q / nodes_[i].thermal_capacity;
		if(!nodes_[i].connections_emissive[k].infinite_thermal_sink)
			dtempdt[nodes_[i].connections_emissive[k].index] += Q/ nodes_[nodes_[i].connections_emissive[k].index].thermal_capacity;


	}
	void thermal_model::fan_convective_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps)
	{
		
		//Convection is complicated	
		//TODO: fix
	}


	//Generate function by using a combination of for loops and switch statements

	void thermal_model::iterate_model(const double dt, const double current, const double voltage, const double rps, const std::vector<identified_val> &assign_temp)
	{
		const double power = current * voltage;
				
		fan_air_speed_ = air_speed_curve_(rps);

		distribute_losses(power, rps);

		//boost::function<void (const ode_state_type &, ode_state_type &, const double )> f2(  );


        rk_stepper.do_step(boost::bind(&thermal_model::compute_coupled_ode_deriv, this, _1, _2, _3 ), temperatures_, 0.0, dt);


	}
	void thermal_model::distribute_losses(const double power, const double rps)
	{
		const double total_electrical_loss = power * efficiency_curve_(rps) * properties_.proportion_losses_electrical;
		const double total_mechanical_loss = power * efficiency_curve_(rps) * properties_.proportion_losses_mechanical;
		for(size_t i = 0; i < nodes_.size(); i++)
		{ 
			const_dtempdt_adder_[i] = nodes_[i].proportion_electrical_loss_absorb * total_electrical_loss; 
			const_dtempdt_adder_[i] += nodes_[i].proportion_mechanical_loss_absorb * total_mechanical_loss; 
		}
	}
	/*
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
	*/
}
