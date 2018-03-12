#include "compressor_control_node/regulate_compressor.h"

static ros::Publisher CompressorCommand;
static ros::Subscriber pressure_sub_;
static ros::Subscriber current_sub_;
static ros::Subscriber match_data_sub_;
static ros::Subscriber disable_sub_;

static double pressure_ = 120;
static double match_time_ = 0;
static bool fms_connected_ =  false;
static double weighted_average_current_ = 0;
static int game_mode_ = 1; //0 for auto, one or greater for anything else
static bool disable_ = false;

int main(int argc, char **argv) {
	ros::init(argc, argv, "compressor_regulator");
	ros::NodeHandle n;

	ros::NodeHandle n_params(n, "model_param");
	double current_multiplier_;
	double pressure_exponent_;
	double pressure_multiplier_;
	double inertial_multiplier_;
	double tank_count_;
	double tank_volume_;
	double max_end_game_use_;
	double max_match_non_end_use_;
	double target_final_pressure_;

	n_params.getParam("current_multiplier", current_multiplier_);
	n_params.getParam("pressure_exponent", pressure_exponent_);
	n_params.getParam("pressure_multiplier", pressure_multiplier_);
	n_params.getParam("inertial_multiplier", inertial_multiplier_);
	n_params.getParam("tank_count", tank_count_);
	n_params.getParam("tank_volume", tank_volume_);
	n_params.getParam("max_end_game_use", max_end_game_use_);
	n_params.getParam("max_match_non_end_use", max_match_non_end_use_);
	n_params.getParam("target_final_pressure", target_final_pressure_);

	max_end_game_use_ *= 60/(tank_count_ * tank_volume_); //converting into tank pressure
	max_match_non_end_use_ *= 60/(tank_count_ * tank_volume_); //converting into tank pressure

	CompressorCommand = n.advertise<std_msgs::Float64>("/frcrobot/compressor_controller/command", 1);

	pressure_sub_ = n.subscribe("/frcrobot/joint_states", 1, &pressureCallback);
	current_sub_ = n.subscribe("/frcrobot/pdp_states", 75, &currentCallback);
	match_data_sub_ = n.subscribe("/frcrobot/match_data", 1, &matchDataCallback);
	disable_sub_ = n.subscribe("/frcrobot/regulate_compressor/disable", 5, &disableCallback);
	//TODO FIX ABOVE topic names

	// TODO : fix me.  spin() will loop forever until !ros::ok()
	// This needs to be a spinOnce at the end of the ros::ok()
	// loop below instead

	ros::Rate r(1); //1 hz
	bool run_last_tick;
	while(ros::ok())
	{
		ros::spinOnce();
		std_msgs::Float64 holder_msg;
		if(fms_connected_ && !disable_ )
		{
			if(game_mode_ != 0 && match_time_ > 30 && (pressure_ < 110 || (run_last_tick && pressure_ < 120)))
			{
				//const double sensor_estimated = (match_time_-30) * (120 - pressure_) / (150 - match_time_);
				//FIX ABOVE SO IT TAKES INTO ACCOUNT REFILLS, and maybe use?
				const double sensor_estimated = 0;
				double max_estimated = max_match_non_end_use_ * (match_time_ - 30)/(120);
				if(sensor_estimated > max_estimated)
				{
					max_estimated = sensor_estimated;
				}
				const double end_pressure_estimate = pressure_ - max_estimated  - max_end_game_use_;
				if(end_pressure_estimate < target_final_pressure_ || run_last_tick)
				{
					const double modelVal = /*-current_multiplier_ * weighted_average_current_
					*/ pressure_multiplier_ * pow(fabs(target_final_pressure_ - 
					end_pressure_estimate),	pressure_exponent_) * ((end_pressure_estimate < 
					target_final_pressure_) ? 1 : -1)
						+ (run_last_tick ? 1 : 0) * inertial_multiplier_;

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
		else if(pressure_ < 100 || run_last_tick && pressure_ < 120)
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
	static int pressure_sensor_index_ = -1;
	if(pressure_sensor_index_ < 0)
	{
		for(size_t i = 0; i < pressure.name.size(); i++)
		{
			if(pressure.name[i] == "analog_pressure")
			{
				pressure_sensor_index_ = i;
				break;
			}
		}
	}
	else
	{
		pressure_ = pressure.position[pressure_sensor_index_];
	}
}
void matchDataCallback(const ros_control_boilerplate::MatchSpecificData &MatchData)
{
	match_time_ = MatchData.matchTimeRemaining;
	fms_connected_ = MatchData.matchTimeRemaining >= 0;
}

// consider a boost::circular_buffer
void currentCallback(const pdp_state_controller::PDPData &msg)
{
	static std::vector<double> currents;

	if(currents.size() > 500)
		currents.erase(currents.begin());

	currents.push_back(msg.totalCurrent);

	double temp_weighted_average_current_ = 0;
	int divider = currents.size() * (1 + currents.size())/2;
	for(size_t i = 0; i < currents.size(); i++)
	{
		temp_weighted_average_current_ += currents[i] * (i+1.0)/divider;
	}
	weighted_average_current_ = temp_weighted_average_current_;
}
void disableCallback(const std_msgs::Bool &disable)
{
	disable_ = disable.data;
}
