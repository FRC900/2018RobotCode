#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>

#include <thermal_modeling/thermal_model.h>

namespace thermal_modeling
{


	thermal_model::thermal_model(std::map<std::string, node_properties> nodes, std::array<std::vector<double>, 2> air_speed_vs_rps, motor_properties properties, std::vector<double> initial_temperatures):
		properties_(properties),
		temperatures_(initial_temperatures)
	{
		
		oh_shittttttttttttttt_ = false;		


		//ROS_ERROR("in const");	
		for(auto &node : nodes)
		{ 
			nodes_.push_back(node.second);
			node_indexes_.insert(std::pair<std::string, int>(node.first, nodes_.size() - 1));
		}
	
		
		//ROS_ERROR_STREAM("_________________________________________________________________________________________________________________________                1: " << node_indexes_["test_lump1"]);
		//ROS_ERROR_STREAM("_________________________________________________________________________________________________________________________                2: " << node_indexes_["test_lump2"]);

		//ROS_ERROR("1");
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
				{
					nodes_[j].connections_conductive[k].index = node_indexes_[nodes_[j].connections_conductive[k].id];
					//ROS_ERROR_STREAM("index: " << nodes_[j].connections_conductive[k].index << " name: " << nodes_[j].connections_conductive[k].id);
				}
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
		
		armature_id_ = node_indexes_[properties.armature_name];
		brush_id_ = node_indexes_[properties.brush_name];
			

		for(auto &bearing_name : properties.bearing_names)
		{

			bearing_ids_.push_back(node_indexes_[bearing_name]);
		
		}
		//ROS_ERROR("2");
		air_speed_curve_.set_points(air_speed_vs_rps[0], air_speed_vs_rps[1]);	
		//ROS_ERROR("3");
	
		
	}
	void thermal_model::compute_coupled_ode_deriv(const ode_state_type &temps, ode_state_type &dtempdt, const double /* t */)
	{
		//ROS_ERROR_STREAM("gets here");	
		for(int i =  0; i < dtempdt.size(); i++)
		{	
			dtempdt[i] = const_dtempdt_adder_[i];	

		}
			
		if(fabs(output_voltage_) < 0.1)
		{
			output_voltage_ = 0.1; //Some value other than 0
			current_term_ = 0;
		} 
		const double predicted_resistance =  (1 + properties_.copper_resistance_alpha * (temps[armature_id_] -  333.15 ))  * properties_.armature_resistance_0v / pow(fabs(output_voltage_), properties_.voltage_exponent);  //(properties_.armature_resistance_12v - properties_.armature_resistance_0v) / (12) * fabs(output_voltage) + properties_.armature_resistance_0v;
		dtempdt[armature_id_] += current_term_ * predicted_resistance / nodes_[armature_id_].thermal_capacity;
		//ROS_ERROR_STREAM("2.1");	

		for(int i =  0; i < dtempdt.size(); i++)
		{ 
			//ROS_ERROR_STREAM("2.15");	
			for(size_t k = 0; k < nodes_[i].connections_emissive.size(); k++)
			{
				//ROS_INFO("emissive deriv");
				emissive_deriv(i, k, dtempdt, temps);
			}
			//ROS_ERROR_STREAM("2.175");	
			for(size_t k = 0; k < nodes_[i].connections_conductive.size(); k++)
			{
				//ROS_INFO("conductive deriv");
				conductive_deriv(i, k, dtempdt, temps);
			}
			//ROS_ERROR_STREAM("2.2");	
			for(size_t k = 0; k < nodes_[i].connections_natural_convective.size(); k++)
			{
				//ROS_INFO("convective deriv");
				natural_convective_deriv(i, k, dtempdt, temps);
			}
			//ROS_ERROR_STREAM("2.3");	
			for(size_t k = 0; k < nodes_[i].connections_fan_convective.size(); k++)
			{
				//ROS_INFO("fan convective deriv");
				fan_convective_deriv(i, k, dtempdt, temps);
			}
			//ROS_ERROR_STREAM("2.4");	
		}
		//ROS_ERROR_STREAM("gets end");	
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

		//ROS_ERROR("2.18");

		const double other_temp = nodes_[i].connections_conductive[k].infinite_thermal_sink ? 
		nodes_[i].connections_conductive[k].infinite_thermal_sink_temp : 
		temps[nodes_[i].connections_conductive[k].index];
		
		//ROS_ERROR("2.185");
		const double Q = nodes_[i].connections_conductive[k].h * (temps[i] - other_temp);
	

	
		//ROS_ERROR("2.19");
		dtempdt[i] -= Q / nodes_[i].thermal_capacity;
		if(!nodes_[i].connections_conductive[k].infinite_thermal_sink)
			dtempdt[nodes_[i].connections_conductive[k].index] += Q/ nodes_[nodes_[i].connections_conductive[k].index].thermal_capacity;


		//ROS_ERROR("2.195");
	}
	void thermal_model::natural_convective_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps)
	{
		const double other_temp = nodes_[i].connections_natural_convective[k].infinite_thermal_sink ? 
		nodes_[i].connections_natural_convective[k].infinite_thermal_sink_temp : 
		temps[nodes_[i].connections_natural_convective[k].index];
		
		const double Q = nodes_[i].connections_natural_convective[k].h * (temps[i] - other_temp);
			

		if(nodes_[i].connections_natural_convective[k].infinite_thermal_sink)
		{
		
			//ROS_INFO_STREAM("q: " << Q << " h: " << nodes_[i].connections_natural_convective[k].h << " diff: " << (temps[i] - other_temp) << " i: "  << i << " k: " << k);
	
		}		


		dtempdt[i] -= Q / nodes_[i].thermal_capacity;
		if(!nodes_[i].connections_natural_convective[k].infinite_thermal_sink)
			dtempdt[nodes_[i].connections_natural_convective[k].index] += Q/ nodes_[nodes_[i].connections_natural_convective[k].index].thermal_capacity;


	}
	void thermal_model::fan_convective_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps)
	{
		const double other_temp = nodes_[i].connections_fan_convective[k].infinite_thermal_sink ? 
		nodes_[i].connections_fan_convective[k].infinite_thermal_sink_temp : 
		temps[nodes_[i].connections_fan_convective[k].index];
		
		const double Q = (nodes_[i].connections_fan_convective[k].v_term * pow(fabs(speed_),  properties_.v_exp_1) + nodes_[i].connections_fan_convective[k].v_squared_term * pow(fabs(speed_),  properties_.v_exp_2)+ nodes_[i].connections_fan_convective[k].v_cubed_term * pow(fabs(speed_),  properties_.v_exp_3))  * (temps[i] - other_temp);
	
		
		dtempdt[i] -= Q / nodes_[i].thermal_capacity;
		if(!nodes_[i].connections_fan_convective[k].infinite_thermal_sink)
			dtempdt[nodes_[i].connections_fan_convective[k].index] += Q/ nodes_[nodes_[i].connections_fan_convective[k].index].thermal_capacity;
		
		//Convection is complicated	
		//TODO: fix
	}


	//Generate function by using a combination of for loops and switch statements

	void thermal_model::iterate_model(const double dt, const double current_term, const double current, const double output_voltage, const double rps, const std::vector<identified_val> &assign_temp)
	{
				
		//fan_air_speed_ = air_speed_curve_(fabs(rps));
		speed_ = fabs(rps);

		//ROS_ERROR_STREAM("1");	
		
		output_voltage_ = output_voltage;

		current_term_ = current_term;
		distribute_losses(output_voltage, fabs(rps));
		output_current_ = current;

	
		//ROS_ERROR_STREAM("2");	
		//boost::function<void (const ode_state_type &, ode_state_type &, const double )> f2(  );


		std::vector<double> temp_temps(temperatures_);


        rk_stepper.do_step(boost::bind(&thermal_model::compute_coupled_ode_deriv, this, _1, _2, _3 ), temperatures_, 0.0, dt);
		//ROS_ERROR_STREAM("3");	

		oh_shittttttttttttttt_ = false;
		for(size_t i = 0; i < temperatures_.size(); i++)
		{
			if(fabs(temp_temps[i] - temperatures_[i]) > 0.5)
			{
				//ROS_INFO_STREAM("c: " << current << " rps: " << rps << " temp: " << temperatures_[i] << " old: " << temp_temps[i]);
				oh_shittttttttttttttt_ = true;
			}
		}



	}
	void thermal_model::distribute_losses(double output_voltage, const double rps)
	{
		
		for(size_t i = 0; i < const_dtempdt_adder_.size(); i++)
		{
			const_dtempdt_adder_[i] = 0;
		}
	

			
		const double custom_term = fabs(output_voltage) < 0.1 ? (0) : (properties_.custom_v_c * pow(fabs(output_voltage),  properties_.custom_v_pow)  * pow(fabs(output_current_),  properties_.custom_c_pow)  );


		const_dtempdt_adder_[armature_id_] += 
		( properties_.v_term * rps 
		+ properties_.v_squared_term * rps * rps 
		+ properties_.volt_term * fabs(output_voltage) 
		+ properties_.volt_squared_term * fabs(output_voltage) * fabs(output_voltage)
		+ properties_.volt_squared_current * fabs(output_voltage) * fabs(output_voltage) * fabs(output_current_) 
		+ properties_.current_squared_volt * fabs(output_voltage) * fabs(output_current_) * fabs(output_current_) 
		+ custom_term
		) / nodes_[armature_id_].thermal_capacity; //Brush friction?
		const_dtempdt_adder_[brush_id_] += properties_.brush_friction_coeff * rps / nodes_[brush_id_].thermal_capacity; //Check this conversion etc.

		/*if(std::isnan(const_dtempdt_adder_[armature_id_]))
		{
			ROS_ERROR_STREAM(const_dtempdt_adder_[armature_id_]);
			ROS_ERROR_STREAM("rps: " << rps);
			ROS_ERROR_STREAM("output_current_: " << output_current_);
			ROS_ERROR_STREAM("output_voltage: " << output_voltage);
		}*/		
		const double bearing_term = properties_.bearing_friction_coeff * rps;

		for(size_t i = 0; i < bearing_ids_.size(); i++)
		{
			const_dtempdt_adder_[bearing_ids_[i]] += bearing_term / nodes_[bearing_ids_[i]].thermal_capacity;
		}

		
			//if(const_dtempdt_adder_[0] > 1.0 || oh_shittttttttttttttt_)
			//	ROS_WARN_STREAM("0: " << const_dtempdt_adder_[0] << " temp: " << temperatures_[0] << " current: " << current<< " rps " << rps);
			//if(const_dtempdt_adder_[1] > 1.0 || oh_shittttttttttttttt_)
			//	ROS_WARN_STREAM("1: " << const_dtempdt_adder_[1]);
		
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
