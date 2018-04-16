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

	connection_type string_to_connection_type(std::string connection_type_string);
	
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
		double emissivity; //0 - 1
		//Add vals here
		connection_emissive():
			emissiveity(0)
			 //initialize vals
		{
		}

	};
	struct connection_conductive : connection_base
	{
		//Add vals here
		connection_conductive() //initialize vals
		{
		}

	};
	struct connection_convective : connection_base
	{
		//Add more vals here
		double exposure; //total exposed area - m^2
		double air_speed; //total area in "fan stream" - m^2	
		connection_convective():
			exposure(0),
			air_speed(0)
		{
		}

	};
	struct node_properties
	{	
		double thermal_capacity; //i.e. J/K
		std::vector<connection_emissive> connections_emissive; //See above struct
		std::vector<connection_conductive> connections_conductive; //See above struct
		std::vector<connection_convective> connections_convective; //See above struct
		double proportion_electrical_loss_absorb; //total should add to 1
		double proportion_mechanical_loss_absorb; //total should add to 1
		double temperature; //needs to be initialized like the others
		node_properties():
			thermal_capacity(0),
			proportion_electrical_loss_absorb(0),
			proportion_mechanical_loss_absorb(0),
			temperature(0)
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
			thermal_model(std::map<std::string, node_properties> nodes, std::array<std::vector<double>, 2> efficiency_vs_rps, std::array<std::vector<double>, 2> air_speed_vs_rps, motor_properties properties); 
			vector<node_properties> nodes_; //Public so info can be read and potentially written to
			std::map<std::string, int> node_indexes_; //We have both a map and a vector for efficiency reasons
			void iterate_model(const double &dt, const double &current, const double &voltage, const double &rps, std::vector<identified_val> assign_temp = {});	

		private: 
			void distribute_losses(const double &dt, const double &rotation_rate, const double &power);
			void diffuse_heat(const double &dt, const double &rotation_rate);
			void distribute_heat();
			tk::spline efficiency_curve_;
			tk::spline air_speed_curve_;
			std::vector<double>  heats_;
			motor_properties properties_;
	};
}
