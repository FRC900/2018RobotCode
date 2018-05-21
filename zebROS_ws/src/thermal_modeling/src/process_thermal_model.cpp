#include <thermal_modeling/thermal_model.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <talon_state_controller/TalonState.h>
#include <string>
#include <boost/numeric/odeint.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <thermal_modeling/ModelTest.h>

using namespace thermal_modeling;

std::vector<std::vector<std::shared_ptr<thermal_modeling::thermal_model>>> motor_models;
std::vector<std::vector<std::string>> talon_names;
std::vector<std::vector<int>> talon_indexes;
std::vector<std::map<std::string, thermal_modeling::node_properties>> all_nodes;
std::vector<std::array<std::vector<double>, 2>> all_air_speed_vs_rps;
std::vector<thermal_modeling::motor_properties> all_properties;
std::vector<std::vector<double>> all_initial_temps;

ros::Subscriber talon_states_sub;
std::vector<ros::ServiceServer> run_model_set;
ros::Publisher motor_limits;



bool run_model_service(thermal_modeling::ModelTest::Request &req, thermal_modeling::ModelTest::Response &res)
{
	
	//ROS_ERROR("_________________________________________________________________         here");


	int talon_index_1 = -1;
	int talon_index_2 = -1;
	for(size_t i = 0; i < motor_models.size(); i++)
    {
        for(size_t k = 0; k < talon_names[i].size(); k++)
        {

			if(talon_names[i][k] == req.params.name)
			{
				talon_index_1 = i;
				talon_index_2 = k;

			}

		}

	}
	//ROS_ERROR("_________________________________________________________________         here1");
	if(talon_index_1  == -1)
	{
		return false;
	}
	std::shared_ptr<thermal_modeling::thermal_model> temp_motor_model;

	thermal_modeling::motor_properties temp_properties = all_properties[talon_index_1];

	
	//ROS_ERROR("_________________________________________________________________         here2");

	std::map<std::string, thermal_modeling::node_properties> temp_nodes = all_nodes[talon_index_1];

	temp_nodes["test_lump1"].connections_emissive[0].h = req.params.e_term1;
	temp_nodes["test_lump1"].connections_conductive[0].h = req.params.h_1;
	temp_nodes["test_lump1"].connections_fan_convective[0].v_term = req.params.v_1;
	temp_nodes["test_lump1"].connections_fan_convective[0].v_squared_term = req.params.v_squared_1;

	//ROS_ERROR("_________________________________________________________________         here3");
	
	temp_nodes["test_lump2"].connections_emissive[0].h = req.params.e_term2;
	//ROS_ERROR("_________________________________________________________________         here3");
	temp_nodes["test_lump2"].connections_natural_convective[0].h = req.params.h_2;
	//ROS_ERROR("_________________________________________________________________         here3");
	temp_nodes["test_lump2"].connections_fan_convective[0].v_term = req.params.v_2;
	//ROS_ERROR("_________________________________________________________________         here3");
	temp_nodes["test_lump2"].connections_fan_convective[0].v_squared_term = req.params.v_squared_2;
	//ROS_ERROR("_________________________________________________________________         here3");

		temp_properties.armature_resistance_12v = req.params.loss_resistance_12v;
		temp_properties.armature_resistance_0v = req.params.loss_resistance_0v;
		temp_properties.v_squared_term = req.params.loss_v_squared_term;
		temp_properties.v_term = req.params.loss_v_term;
		temp_properties.volt_squared_term = req.params.loss_volt_squared_term;
		temp_properties.volt_term = req.params.loss_volt_term;
		temp_properties.voltage_exponent = req.params.voltage_exponent;
		temp_properties.volt_squared_current  = req.params.volt_squared_current;	
		temp_properties.current_squared_volt  = req.params.current_squared_volt;	
		temp_properties.custom_v_c  = req.params.custom_v_c;
		temp_properties.custom_c_pow  = req.params.custom_v_pow;
		temp_properties.custom_v_pow  = req.params.custom_c_pow;


	//ROS_ERROR("_________________________________________________________________         here4");

	std::vector<double> temp_initial_temps = req.params.initial_temps;
		
	

	

	temp_motor_model = std::make_shared<thermal_modeling::thermal_model>(temp_nodes, all_air_speed_vs_rps[talon_index_1], temp_properties, temp_initial_temps);		

	
	res.temps.resize(temp_motor_model->temperatures_.size());
	for(size_t i = 0; i < temp_motor_model->temperatures_.size(); i++)
	{	
		res.temps[i].temps.resize(req.time.size());

	}
	for(size_t i = 1; i < req.time.size(); i++)
	{
		//ROS_ERROR_STREAM("t2: " << temp_motor_model->temperatures_[1]);

		double current_term = fabs(req.output_current[i]) *fabs(req.output_current[i])  * fabs(req.bus_voltage[i]);





		temp_motor_model->iterate_model(req.time[i] - req.time[i-1], current_term, fabs(req.output_current[i]), fabs(req.output_voltage[i]), fabs(req.RPM[i]));
		for(size_t k = 0; k < temp_motor_model->temperatures_.size(); k++)
		{	
			res.temps[k].temps[i] = temp_motor_model->temperatures_[k];
			
		}
		
	}
	
}
void talon_cb(const talon_state_controller::TalonState &msg)
{
	static double time_p = msg.header.stamp.toSec();
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
					time_p = msg.header.stamp.toSec();
					return;
				}

			}
			double dt = msg.header.stamp.toSec() - time_p;
			

			double current_term = fabs(msg.output_current[talon_indexes[i][k]]) *fabs(msg.output_current[talon_indexes[i][k]])  * fabs(msg.bus_voltage[talon_indexes[i][k]]) / fabs(msg.output_voltage[talon_indexes[i][k]]);
	

			//ROS_ERROR("actually running");
			motor_models[i][k]->iterate_model(dt, current_term, fabs(msg.output_current[talon_indexes[i][k]]), fabs(msg.output_voltage[talon_indexes[i][k]]), fabs(msg.speed[talon_indexes[i][k]]));
			//ROS_ERROR("1.2.2");

				
			static int count = 0;
			if(count % 500 == 0)
			{
				ROS_ERROR_STREAM(msg.header.stamp.toSec());
				ROS_ERROR_STREAM("t1: " << motor_models[i][k]->temperatures_[0] << " t2: " <<  motor_models[i][k]->temperatures_[1]);
			}
			count++;


		}
		
		//TODO: publish results
	}
	time_p = msg.header.stamp.toSec();

	
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

	all_nodes.resize(num_motor_types);
	all_air_speed_vs_rps.resize(num_motor_types);
	all_properties.resize(num_motor_types);
	all_initial_temps.resize(num_motor_types);


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
		
		XmlRpc::XmlRpcValue node_names_xml;
		

		if(!motor_params.getParam("node_names", node_names_xml))
			ROS_ERROR_STREAM("Could not get node_names in process thermal model motor type: " << motor_types_xml[i]);	
		all_initial_temps[i].resize(node_names_xml.size());
		for(size_t k = 0; k < node_names_xml.size(); k++)
		{
			ros::NodeHandle node_params(motor_params, node_names_xml[k]);

			thermal_modeling::node_properties node;
			
			if(!node_params.getParam("thermal_capacity", node.thermal_capacity))
				ROS_ERROR_STREAM("Could not get thermal_capacity in process thermal model motor type: " << motor_types_xml[i] << " node: " <<node_names_xml[k]);	
			if(!node_params.getParam("initial_temperature", all_initial_temps[i][k]))
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
					ROS_ERROR_STREAM("t: " << temp_connection.infinite_thermal_sink_temp);

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
				thermal_modeling::connection_fan_convective temp_connection;
				temp_connection.v_term = fan_convective_connections_xml[j]["v_term"];
				temp_connection.v_squared_term = fan_convective_connections_xml[j]["v_squared_term"];
			

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


			all_nodes[i].insert(std::pair<std::string, thermal_modeling::node_properties>(node_names_xml[k], node));	
		}
		/*std::array<std::vector<double>, 2> efficiency_vs_rps;
	
		
		XmlRpc::XmlRpcValue efficiency_vs_rps_xml;
	
		if(!motor_params.getParam("efficiency_vs_rps", efficiency_vs_rps_xml))
			ROS_ERROR_STREAM("Could not get efficiency_vs_rps in process thermal model motor type: " << motor_types_xml[i]);	
		
		efficiency_vs_rps[0].resize(efficiency_vs_rps_xml.size());
		efficiency_vs_rps[1].resize(efficiency_vs_rps_xml.size());
		for(size_t k = 0; k < efficiency_vs_rps_xml.size(); k++)
		{
			efficiency_vs_rps[0][k] = efficiency_vs_rps_xml[k]["rps"];
			efficiency_vs_rps[1][k] = efficiency_vs_rps_xml[k]["eff"];
		}
		*/

	
		
		XmlRpc::XmlRpcValue air_speed_vs_rps_xml;
	
		if(!motor_params.getParam("air_speed_vs_rps", air_speed_vs_rps_xml))
			ROS_ERROR_STREAM("Could not get air_speed_vs_rps in process thermal model motor type: " << motor_types_xml[i]);	
		

		all_air_speed_vs_rps[i][0].resize(air_speed_vs_rps_xml.size());
		all_air_speed_vs_rps[i][1].resize(air_speed_vs_rps_xml.size());
		for(size_t k = 0; k < air_speed_vs_rps_xml.size(); k++)
		{	
			all_air_speed_vs_rps[i][0][k] = air_speed_vs_rps_xml[k]["rps"];
			all_air_speed_vs_rps[i][1][k] = air_speed_vs_rps_xml[k]["speed"];
		}


		XmlRpc::XmlRpcValue properties_xml;

		if(!motor_params.getParam("properties", properties_xml))
			ROS_ERROR_STREAM("Could not get properties in process thermal model motor type: " << motor_types_xml[i]);	
	
		all_properties[i].armature_resistance_12v = properties_xml["armature_resistance_12v"];	
		all_properties[i].armature_resistance_0v = properties_xml["armature_resistance_0v"];	
		all_properties[i].brush_friction_coeff = properties_xml["brush_friction_coeff"];	
		all_properties[i].bearing_friction_coeff = properties_xml["bearing_friction_coeff"];	
		all_properties[i].v_squared_term = properties_xml["v_squared_term"];	
		all_properties[i].v_term = properties_xml["v_term"];	
		all_properties[i].volt_squared_term = properties_xml["volt_squared_term"];	
		all_properties[i].volt_term = properties_xml["volt_term"];	
		all_properties[i].volt_squared_current  = properties_xml["volt_squared_current"];	
		all_properties[i].current_squared_volt  = properties_xml["current_squared_volt"];	
		all_properties[i].custom_v_c  = properties_xml["custom_v_c"];
		all_properties[i].custom_v_pow  = properties_xml["custom_v_pow"];
		all_properties[i].custom_c_pow  = properties_xml["custom_c_pow"];
		all_properties[i].voltage_exponent = properties_xml["voltage_exponent"];	
		all_properties[i].copper_resistance_alpha = properties_xml["copper_resistance_alpha"];	
		all_properties[i].armature_name = static_cast<std::string>(properties_xml["armature_name"]);	
		all_properties[i].brush_name =  static_cast<std::string>(properties_xml["brush_name"]);	
		
		for(size_t n = 0; n < properties_xml["bearing_names"].size(); n++)
		{

			all_properties[i].bearing_names.push_back(static_cast<std::string>(properties_xml["bearing_names"][n]));	

		}
		
		motor_models[i].resize(talon_names_xml.size());

		//ROS_ERROR("10");
		for(size_t k = 0; k < talon_names_xml.size(); k++)
		{	
			//ROS_ERROR_STREAM("k of " << k);
			//motor_models[i][k];
			//ROS_ERROR("be");


			motor_models[i][k] = std::make_shared<thermal_modeling::thermal_model>(all_nodes[i], all_air_speed_vs_rps[i], all_properties[i], all_initial_temps[i]);		
		}
		//ROS_ERROR("9");
	
	}
		
	talon_states_sub = n.subscribe("/frcrobot/talon_states", 8000, &talon_cb);	
	
	int num_servers;

	if(!n_params.getParam("num_servers", num_servers))
        ROS_ERROR("Could not get num_servers in process thermal model");

	run_model_set.resize(num_servers);
	for(int i = 0; i < num_servers; i++)
	{
		run_model_set[i] = n.advertiseService("/frcrobot/thermal_test" + std::to_string(i),  &run_model_service);

	}	


	ROS_WARN("spinning - thermal");
	ros::spin();

	return 0;
}
