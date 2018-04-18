#pragma once

#include <cmath>
#include <thermal_modeling/spline.h>
#include <thermal_modeling/rk4.hpp>
#include <vector>
#include <array>
#include <string>
#include <map>
#include <memory>
#include <cstdio> //Consider changing this to ros consoole stuff

//Simpifying assumptions
//
// * Fan air flow is limited to specific subset of locations, assumed same speed between locations, air speed is assumed proportional to motor speed
// * Properties remain constant with temperature (for now)



//Units: K (kelvin), rps (radians per second), m (meters), ...

namespace thermal_modeling
{

	
	typedef std::vector<double> ode_state_type;

	
	struct identified_val
	{
		double val;
		std::string id;
		identified_val():
			val(0),
			id("id_not_initialized")
		{
		}
	};
	struct connection_base
	{
		bool infinite_thermal_sink;
		double infinite_thermal_sink_temp; //doesn't need to be set unless above is true
										   //Most of the time should just be environ temp
		std::string id;
		int index; //Doesn't need to be filled out on input
		connection_base():
			infinite_thermal_sink(false),
			id("id_not_initialized"),
			index(-1)
		{
		}
	};
	struct connection_emissive : connection_base
	{
		double h;	
	
		connection_emissive(
		const double emissivity_here, //0 - 1
        const double emissivity_there,
        const double area_here,
        const double area_there,
        const double view_factor //from here to there
		)
		{
			h = 5.670400e-8 /*Stefan Boltzmann Constant*/ / ((1.-emissivity_here)/(area_here * emissivity_here ) + (1.) / 
			(area_here * view_factor) + (1.-emissivity_there)/(area_there * emissivity_there ));
		}

	};
	struct connection_conductive : connection_base
	{
		//Add vals here
		double h;
		connection_conductive(
		const double k,
		const double area,
		const double dist
		) 
		{
			h = k*area / dist;
		}

	};
	struct connection_natural_convective : connection_base
	{
		double h;
		//Add more vals here
		//double exposure; //total exposed area - m^2
		//double air_speed; //total area in "fan stream" - m^2	
		connection_natural_convective(
		const double k,
		const double area
		)
			//exposure(0),
			//air_speed(0)
		{
			h = k * area;
		}

	};
	struct connection_fan_convective : connection_base
	{
		//TODO: fix this
		double h; 
		//Add more vals here
		//double exposure; //total exposed area - m^2
		//double air_speed; //total area in "fan stream" - m^2	
		connection_fan_convective(
		const double k,
		const double area
		)
			//exposure(0),
			//air_speed(0)
		{
			h = k * area;
		}

	};
	struct node_properties
	{	
		double thermal_capacity; //i.e. J/K
		std::vector<connection_emissive> connections_emissive; //See above struct
		std::vector<connection_conductive> connections_conductive; //See above struct
		std::vector<connection_natural_convective> connections_natural_convective; //See above struct
		std::vector<connection_fan_convective> connections_fan_convective; //See above struct
		double proportion_electrical_loss_absorb; //total should add to 1
		double proportion_mechanical_loss_absorb; //total should add to 1
		node_properties():
			thermal_capacity(0),
			proportion_electrical_loss_absorb(0),
			proportion_mechanical_loss_absorb(0)
		{
		}
	};
	struct motor_properties
	{
		double proportion_losses_mechanical; //Consider making this more complex
		double proportion_losses_electrical;	
		motor_properties():
			proportion_losses_mechanical(0),
			proportion_losses_electrical(0)
		{
		}
	};
	/*struct point
	{
		double x;
		double y;
	};*/
	//TODO: constructors
	class thermal_model
	{
		public:
			thermal_model(std::map<std::string, node_properties> nodes, std::array<std::vector<double>, 2> efficiency_vs_rps, std::array<std::vector<double>, 2> air_speed_vs_rps, motor_properties properties, std::vector<double> initial_temperatures); 
			std::vector<node_properties> nodes_; //Public so info can be read and potentially written to
			std::vector<double> temperatures_; //needs to be initialized like the others
			std::map<std::string, int> node_indexes_; //We have both a map and a vector for efficiency reasons
			void iterate_model(const double dt, const double current, const double voltage, const double rps, const std::vector<identified_val> &assign_temp = {});	

		private: 
			void distribute_losses(const double power, const double rps);
			void compute_coupled_ode_deriv(const ode_state_type &temps, ode_state_type &dtempdt, 
			const double t);
			void emissive_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps);
			void natural_convective_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps);
			void fan_convective_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps);
			void conductive_deriv(const int i, const int k, ode_state_type &dtempdt, const ode_state_type &temps);
			std::vector<double> const_dtempdt_adder_;
			//void diffuse_heat(const double &dt, const double &rotation_rate);
			//void distribute_heat();
			tk::spline efficiency_curve_;
			tk::spline air_speed_curve_;
			double fan_air_speed_;
			motor_properties properties_;
	};
}
