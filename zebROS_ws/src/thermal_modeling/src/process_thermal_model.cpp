#include <thermal_modeling/thermal_model.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <talon_state_controller/TalonState.h>
#include <string>
#include <boost/numeric/odeint.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>


using namespace thermal_modeling;

std::vector<std::vector<std::shared_ptr<thermal_modeling::thermal_model>>> motor_models;
std::vector<std::vector<std::string>> talon_names;
std::vector<std::vector<int>> talon_indexes;


ros::Subscriber talon_states_sub;
ros::Publisher motor_limits;

void talon_cb(const talon_state_controller::TalonState &msg)
{
	static double time_p = ros::Time::now().toSec();
	for(size_t i = 0; i < motor_models.size(); i++)
	{
		for(size_t k = 0; k < talon_names[i].size(); k++)
		{
			if(talon_indexes[i][k] == -1)
			{
				for(size_t j = 0; j < msg.name.size(); j++)
				{
					//ROS_INFO_STREAM("name: " << msg.name[j]);
					if(msg.name[j] == talon_names[i][k])
					{
						talon_indexes[i][k] = j;
						break;
					}

				}
				if(talon_indexes[i][k] == -1)
				{
					
					ROS_ERROR_STREAM(" talon with name: " << talon_names[i][k] << " not found");
					time_p = ros::Time::now().toSec();
					return;
				}

			}
			double dt = ros::Time::now().toSec() - time_p;
			ROS_ERROR("actually running");
			motor_models[i][k]->iterate_model(dt, msg.output_current[talon_indexes[i][k]], msg.output_voltage[talon_indexes[i][k]], fabs(msg.speed[talon_indexes[i][k]]));
	


		}
		
		//TODO: publish results
	}
	time_p = ros::Time::now().toSec();

	
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "process_thermal_model");
	ros::NodeHandle n;
		
	ros::NodeHandle n_params(n, "thermal_model_params");
	
	XmlRpc::XmlRpcValue motor_types_xml;
	if(!n_params.getParam("motor_types", motor_types_xml))
		ROS_ERROR("Could not get motor_types in process thermal model");
	//TODO: add error checking on the type of motor_types_xml (hypothetically vector of strings)
	//And error checking on all below types
	int num_motor_types = motor_types_xml.size();
	motor_models.resize(num_motor_types);
	talon_names.resize(num_motor_types);
	talon_indexes.resize(num_motor_types);
	ROS_WARN("------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------before loop");
	for(size_t i = 0; i < num_motor_types; i++)
	{
		ros::NodeHandle motor_params(n_params, motor_types_xml[i]);
		XmlRpc::XmlRpcValue talon_names_xml;
		
		if(!motor_params.getParam("talon_names", talon_names_xml))
			ROS_ERROR_STREAM("Could not get talon_names in process thermal model motor type: " << motor_types_xml[i]);	
		for(size_t k = 0; k < talon_names_xml.size(); k++)
		{
			talon_names[i].push_back(talon_names_xml[k]);
			talon_indexes[i].push_back(-1);
		}
		std::map<std::string, thermal_modeling::node_properties> nodes;
		
		XmlRpc::XmlRpcValue node_names_xml;
	ROS_WARN("q------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------before loop");
		

		if(!motor_params.getParam("node_names", node_names_xml))
			ROS_ERROR_STREAM("Could not get node_names in process thermal model motor type: " << motor_types_xml[i]);	
		std::vector<double> initial_temps;
		initial_temps.resize(node_names_xml.size());
		for(size_t k = 0; k < node_names_xml.size(); k++)
		{
			ros::NodeHandle node_params(motor_params, node_names_xml[k]);

			thermal_modeling::node_properties node;
			
			if(!node_params.getParam("thermal_capacity", node.thermal_capacity))
				ROS_ERROR_STREAM("Could not get thermal_capacity in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	
			if(!node_params.getParam("proportion_electrical_loss_absorb", node.proportion_electrical_loss_absorb))
				ROS_ERROR_STREAM("Could not get proportion_electrical_loss_absorb in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	
			if(!node_params.getParam("proportion_mechanical_loss_absorb", node.proportion_mechanical_loss_absorb))
				ROS_ERROR_STREAM("Could not get proportion_mechanical_loss_absorb in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	
			if(!node_params.getParam("initial_temperature", initial_temps[k]))
				ROS_ERROR_STREAM("Could not get initial_temperature in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	
				
	        XmlRpc::XmlRpcValue emissive_connections_xml;
			if(!node_params.getParam("emissive_connections", emissive_connections_xml))
				ROS_ERROR_STREAM("Could not get emissive_connections in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	

			for(size_t j = 0; j < emissive_connections_xml.size(); j++)
			{
				thermal_modeling::connection_emissive temp_connection(emissive_connections_xml[j]["emissivity_here"], emissive_connections_xml[j]["emissivity_there"],  emissive_connections_xml[j]["area_here"], emissive_connections_xml[j]["area_there"],  emissive_connections_xml[j]["view_factor"]);

				temp_connection.infinite_thermal_sink = emissive_connections_xml[j]["infinite_thermal_sink"];
				if(temp_connection.infinite_thermal_sink)
				{
					temp_connection.infinite_thermal_sink_temp = emissive_connections_xml[j]["infinite_thermal_sink_temp"];
				}
				else
				{
					temp_connection.id = static_cast<std::string>(emissive_connections_xml[j]["id"]);
				}
				node.connections_emissive.push_back(temp_connection);
			}
	        XmlRpc::XmlRpcValue conductive_connections_xml;
			if(!node_params.getParam("conductive_connections", conductive_connections_xml))
				ROS_ERROR_STREAM("Could not get conductive_connections in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	

			for(size_t j = 0; j < conductive_connections_xml.size(); j++)
			{
				thermal_modeling::connection_conductive temp_connection(conductive_connections_xml[j]["k"], conductive_connections_xml[j]["area"], conductive_connections_xml[j]["dist"]);

				temp_connection.infinite_thermal_sink = conductive_connections_xml[j]["infinite_thermal_sink"];
				if(temp_connection.infinite_thermal_sink)
				{
					temp_connection.infinite_thermal_sink_temp = conductive_connections_xml[j]["infinite_thermal_sink_temp"];
				}
				else
				{	
					temp_connection.id = static_cast<std::string>(conductive_connections_xml[j]["id"]);
				}
				
				node.connections_conductive.push_back(temp_connection);
			}
	ROS_WARN("2------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------before loop");
	        XmlRpc::XmlRpcValue natural_convective_connections_xml;
			if(!node_params.getParam("natural_convective_connections", natural_convective_connections_xml))
				ROS_ERROR_STREAM("Could not get natural_convective_connections in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	

			for(size_t j = 0; j < natural_convective_connections_xml.size(); j++)
			{
				thermal_modeling::connection_natural_convective temp_connection(conductive_connections_xml[j]["k"], conductive_connections_xml[j]["area"]);

				temp_connection.infinite_thermal_sink = natural_convective_connections_xml[j]["infinite_thermal_sink"];
				if(temp_connection.infinite_thermal_sink)
				{
					temp_connection.infinite_thermal_sink_temp = natural_convective_connections_xml[j]["infinite_thermal_sink_temp"];
				}
				else
				{
					temp_connection.id = static_cast<std::string>(natural_convective_connections_xml[j]["id"]);
				}
				node.connections_natural_convective.push_back(temp_connection);
			}
	        XmlRpc::XmlRpcValue fan_convective_connections_xml;
			if(!node_params.getParam("fan_convective_connections", fan_convective_connections_xml))
				ROS_ERROR_STREAM("Could not get fan_convective_connections in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	

			for(size_t j = 0; j < fan_convective_connections_xml.size(); j++)
			{
				thermal_modeling::connection_fan_convective temp_connection(conductive_connections_xml[j]["k"], conductive_connections_xml[j]["area"]);

				temp_connection.infinite_thermal_sink = fan_convective_connections_xml[j]["infinite_thermal_sink"];
				if(temp_connection.infinite_thermal_sink)
				{
					temp_connection.infinite_thermal_sink_temp = fan_convective_connections_xml[j]["infinite_thermal_sink_temp"];
				}
				else
				{
					temp_connection.id = static_cast<std::string>(fan_convective_connections_xml[j]["id"]);
				}
				
				node.connections_fan_convective.push_back(temp_connection);
			}

	ROS_WARN("3------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------before loop");

			nodes.insert(std::pair<std::string, thermal_modeling::node_properties>(node_names_xml[k], node));	
		}
		std::array<std::vector<double>, 2> efficiency_vs_rps;
	
		
		XmlRpc::XmlRpcValue efficiency_vs_rps_xml;
	
		if(!motor_params.getParam("efficiency_vs_rps", efficiency_vs_rps_xml))
			ROS_ERROR_STREAM("Could not get efficiency_vs_rps in process thermal model motor type: " << motor_types_xml[i]);	
		

		for(size_t k = 0; k < efficiency_vs_rps_xml.size(); k++)
		{	
			efficiency_vs_rps[0][k] = efficiency_vs_rps_xml[k]["rps"];
			efficiency_vs_rps[1][k] = efficiency_vs_rps_xml[k]["eff"];
		}
	

		std::array<std::vector<double>, 2> air_speed_vs_rps;
	
		
		XmlRpc::XmlRpcValue air_speed_vs_rps_xml;
	
		if(!motor_params.getParam("air_speed_vs_rps", air_speed_vs_rps_xml))
			ROS_ERROR_STREAM("Could not get air_speed_vs_rps in process thermal model motor type: " << motor_types_xml[i]);	
		

		for(size_t k = 0; k < air_speed_vs_rps_xml.size(); k++)
		{	
			air_speed_vs_rps[0][k] = air_speed_vs_rps_xml[k]["rps"];
			air_speed_vs_rps[1][k] = air_speed_vs_rps_xml[k]["speed"];
		}

		thermal_modeling::motor_properties properties;

		XmlRpc::XmlRpcValue properties_xml;

		if(!motor_params.getParam("properties", properties_xml))
			ROS_ERROR_STREAM("Could not get properties in process thermal model motor type: " << motor_types_xml[i]);	
	
		properties.proportion_losses_mechanical = properties_xml["proportion_losses_mechanical"];	
		properties.proportion_losses_electrical = properties_xml["proportion_losses_electrical"];	


	ROS_WARN("4------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------before loop");
		motor_models[i].resize(talon_names_xml.size());

		for(size_t k = 0; k < talon_names_xml.size(); k++)
		{	
			motor_models[i][k] = std::make_shared<thermal_modeling::thermal_model>(nodes, efficiency_vs_rps, air_speed_vs_rps, properties, initial_temps);		
		}
	}
	
	talon_states_sub = n.subscribe("/frcrobot/talon_states", 1, &talon_cb);	

	ROS_WARN("spinning");
	ros::spin();

	return 0;
}
