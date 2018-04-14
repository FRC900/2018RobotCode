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
		std::vector<connection> connections; //See above struct
		double emissivity; //0 - 1
		double proportion_electrical_loss_absorb; //total should add to 1
		double proportion_mechancal_loss_absorb; //total should add to 1
		double exposure; //total exposed area - m^2
		double fan_exposure; //total area in "fan stream" - m^2	
		double temperature; //needs to be initialized like the others
	};
	struct motor_properties
	{
		double proportion_losses_mechanical; //Consider making this more complex
		double proportion_losses_electrical;	
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
			thermal_model(std::vector<node_properties> nodes, std::array<std::vector<double>, 2> efficiency_vs_rps, std::array<std::vector<double>, 2> air_speed_vs_rps, motor_properties properties); 
			std::vector<node_properties> nodes_; //Public so info can be read and potentially written to
			void iterate_model(double dt. double current, double voltage, double rps, std::vector<identified_val> assign_temp = {});
		

		private: 
			void distribute_losses(const double &dt, const double &rotation_rate, const double &power);
			void diffuse_heat(const double &dt, const double &rotation_rate);
			void distribute_heat();
			tk::spline efficiency_curve_;
			tk::spline air_speed_curve_;
			std::vector<double> heats_;
			motor_properties properties_;
	};
}
