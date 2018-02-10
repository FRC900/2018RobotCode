#include "iostream"
#include "pdp_data/PDPData.h" 
#include <ros/ros.h>
#include <thread>
#include <realtime_tools/realtime_publisher.h>
//#include "../../wpilib/cpp/current/include/PowerDistributionPanel.h"


int main(int argc, char **argv){
	ros::init(argc, argv, "pdp_data_publisher");
	ros::NodeHandle n;

/*	PowerDistributionPanel pdp(0);
	pdp.ClearStickyFaults();
	pdp.ResetTotalEnergy();

	realtime_tools::RealtimePublisher<pdp_data::PDPData> pdp_data_publisher(nh_, "pdp_data", 4);

	ros::Rate loop_rate(10);

	if(pdp_data_publisher.trylock())
	{
		pdp_data_publisher.msg_.header.stamp = ros::Time::now();

		pdp_data_publisher.msg_.voltage = pdp.GetVoltage ();
		pdp_data_publisher.msg_.temperature = pdp.GetTemperature();
		pdp_data_publisher.msg_.totalCurrent = pdp.GetTotalCurrent()
		pdp_data_publisher.msg_.totalPower = pdp.GetTotalPower();
		pdp_data_publisher.msg_.totalEnergy = pdp.GetTotalEnergy();

		for(int channel = 0; channel < 16; channel++)
		{
			pdp_data_publisher.msg_.current[channel] = pdp.GetCurrent(channel);
		}

		pdp_data_publisher.unlockAndPublish();		

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
	*/
}

