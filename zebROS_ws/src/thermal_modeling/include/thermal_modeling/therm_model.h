#pragma once

#include <cmath>
#include <thermal_modeling/rk4.hpp>
#include <vector>
#include <stdio> //Consider changing this to ros consoole stuff

//Simpifying assumptions
//
// * Fan air flow is limited to specific subset of locations, assumed same speed between locations, air speed is assumed proportional to motor speed
// * Properties remain constant with temperature (for now)



//Units: K (kelvin), rps (radians per second), m (meters), ...

namespace thermal_model
{
	enum connection_type //Add more as needed
	{
		emissive,
		conductive
	};
	struct identified_val
	{
		double val;
		int id;
	};
	struct connection
	{
		int id;
		connection_type type;
		double value; //Units/what this does will depend on the type
	};
	struct node_properties
	{	
		double thermal_capacity; //i.e. J/K
		vector<connection> connections; //See above struct
		double emissivity; //0 - 1
		double percent_electrical_loss_absorb; //total should add to 1
		double percent_mechancal_loss_absorb; //total should add to 1
		double exposure; //total exposed area - m^2
		double fan_exposure; //total area in "fan stream" - m^2	
		double temperature; //needs to be initialized like the others
	};
	struct point
	{
		double x;
		double y;
	};
	//TODO: constructors
	class thermal_model
	{
		public:
			thermal_model(vector<node_properties> nodes, vector<point> efficiency_vs_rps, vector<point> air_speed_vs_rps); 
			vector<node_properties> nodes_; //Public so info can be read and potentially written to
			void iterate_model(double dt. double current, double voltage, double rps, vector<identified_val> assign_temp = {});














	}








}
