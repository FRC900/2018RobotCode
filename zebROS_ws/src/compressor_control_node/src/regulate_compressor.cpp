#include <atomic>
#include "compressor_control_node/regulate_compressor.h"

static std::atomic<double> pressure_;
static std::atomic<double> match_time_;
static std::atomic<bool> fms_connected_;
static std::atomic<double> weighted_average_current_;
static int game_mode_ = 1; //0 for auto, one or greater for anything else
static std::atomic<bool> disable_;

int main(int argc, char **argv) {
	ros::init(argc, argv, "compressor_regulator");
	ros::NodeHandle n;

	ros::NodeHandle n_params(n, "model_params");
	double current_multiplier_;
	double pressure_exponent_;
	double pressure_multiplier_;
	double inertial_multiplier_;
	double tank_count_;
	double tank_volume_;
	double max_end_game_use_;
	double max_match_non_end_use_;
	double target_final_pressure_;

	if (!n_params.getParam("current_multiplier", current_multiplier_))
		ROS_ERROR("Could not read current_multiplier in regulate_compressor");
	if (!n_params.getParam("pressure_exponent", pressure_exponent_))
		ROS_ERROR("Could not read pressure_exponent in regulate_compressor");
	if (!n_params.getParam("pressure_multiplier", pressure_multiplier_))
		ROS_ERROR("Could not read pressure_multiplier in regulate_compressor");
	if (!n_params.getParam("inertial_multiplier", inertial_multiplier_))
		ROS_ERROR("Could not read inertial_multiplier in regulate_compressor");
	if (!n_params.getParam("tank_count", tank_count_))
		ROS_ERROR("Could not read tank_count in regulate_compressor");
	if (!n_params.getParam("tank_volume", tank_volume_))
		ROS_ERROR("Could not read tank_volume in regulate_compressor");
	if (!n_params.getParam("max_end_game_use", max_end_game_use_))
		ROS_ERROR("Could not read max_end_game_use in regulate_compressor");
	if (!n_params.getParam("max_match_non_end_use", max_match_non_end_use_))
		ROS_ERROR("Could not read max_match_non_end_use in regulate_compressor");
	if (!n_params.getParam("target_final_pressure", target_final_pressure_))
		ROS_ERROR("Could not read target_final_pressure in regulate_compressor");

	max_end_game_use_ *= 60./(tank_count_ * tank_volume_); //converting into tank pressure
	max_match_non_end_use_ *= 60./(tank_count_ * tank_volume_); //converting into tank pressure

	ros::Publisher CompressorCommand = n.advertise<std_msgs::Float64>("/frcrobot/compressor_controller/command", 1);

	ros::Subscriber pressure_sub_ = n.subscribe("/frcrobot/joint_states", 1, &pressureCallback);
	ros::Subscriber current_sub_ = n.subscribe("/frcrobot/pdp_states",60, &currentCallback);
	ros::Subscriber match_data_sub_ = n.subscribe("/frcrobot/match_data", 1, &matchDataCallback);
	ros::Subscriber disable_sub_ = n.subscribe("/frcrobot/regulate_compressor/disable", 5, &disableCallback);
	//TODO FIX ABOVE topic names

	pressure_ = 120;
	match_time_ = 0;
	fms_connected_ = false;
	weighted_average_current_ = 0;
	disable_ = false;

	ros::Rate r(1); //1 hz
	bool run_last_tick;
	while(ros::ok())
	{
		ros::spinOnce();
		std_msgs::Float64 holder_msg;
		const double this_match_time = match_time_.load(std::memory_order_relaxed);
		const double this_pressure = pressure_.load(std::memory_order_relaxed);
		if(fms_connected_.load(std::memory_order_relaxed) && !disable_.load(std::memory_order_relaxed) )
		{
			if(game_mode_ != 0 && this_match_time > 30 && (this_pressure < 110 || (run_last_tick && this_pressure < 120)))
			{
				//const double sensor_estimated = (this_match_time-30) * (120 - this_pressure) / (150 - this_match_time);
				//FIX ABOVE SO IT TAKES INTO ACCOUNT REFILLS, and maybe use?
				const double sensor_estimated = 0;
				double max_estimated = max_match_non_end_use_ * (this_match_time - 30)/(120);
				if(sensor_estimated > max_estimated)
				{
					max_estimated = sensor_estimated;
				}
				const double end_pressure_estimate = this_pressure - max_estimated  - max_end_game_use_;
				if(end_pressure_estimate < target_final_pressure_ || run_last_tick)
				{
					const double modelVal = -current_multiplier_ * weighted_average_current_.load(std::memory_order_relaxed)
					+ pressure_multiplier_ * pow(fabs(target_final_pressure_ - 
					end_pressure_estimate),	pressure_exponent_) * ((end_pressure_estimate < 
					target_final_pressure_) ? 1 : -1)
						+ (run_last_tick ? 1 : 0) * inertial_multiplier_;

					ROS_INFO_STREAM("model val: " << modelVal);
					if(modelVal > 0)
					{
						holder_msg.data = 1;
						run_last_tick = true;
					}
					else
					{
						holder_msg.data = 0;
						run_last_tick = false;
					}
				}
				else
				{
					holder_msg.data = 0;
					run_last_tick = false;
				}
			}
			else
			{
				holder_msg.data = 0;
				run_last_tick = false;

			}
		}
		/*
		else if(this_pressure < 100 || run_last_tick && this_pressure < 120)
		{
			holder_msg.data = 1;
			run_last_tick = true;
		}
		*/
		else
		{
			holder_msg.data = 1;
			run_last_tick = true;
		}

		CompressorCommand.publish(holder_msg);

		r.sleep();
	}

	return 0;
}

void pressureCallback(const sensor_msgs::JointState &pressure)
{
	static int pressure_sensor_index = -1;
	for(size_t i = 0; (pressure_sensor_index < 0) && (i < pressure.name.size()); i++)
		if(pressure.name[i] == "analog_pressure")
			pressure_sensor_index = i;

	if(pressure_sensor_index >= 0)
		pressure_.store(pressure.position[pressure_sensor_index], std::memory_order_relaxed);
}

void matchDataCallback(const ros_control_boilerplate::MatchSpecificData &MatchData)
{
	match_time_.store(MatchData.matchTimeRemaining, std::memory_order_relaxed);
	fms_connected_.store(MatchData.matchTimeRemaining >= 0, std::memory_order_relaxed);
}

// consider a boost::circular_buffer
void currentCallback(const pdp_state_controller::PDPData &msg)
{
	static std::vector<double> currents;

	if(currents.size() > 60)
		currents.erase(currents.begin());

	currents.push_back(msg.totalCurrent);

	double temp_weighted_average_current = 0;
	int divider = currents.size() * (1 + currents.size())/2;
	for(size_t i = 0; i < currents.size(); i++)
	{
		temp_weighted_average_current += currents[i] * (i+1.0)/divider;
	}
	//ROS_INFO_STREAM("current weigted avg: " << temp_weighted_average_current);
	weighted_average_current_.store(temp_weighted_average_current, std::memory_order_relaxed);
}

void disableCallback(const std_msgs::Bool &disable)
{
	disable_.store(disable.data, std::memory_order_relaxed);
}
