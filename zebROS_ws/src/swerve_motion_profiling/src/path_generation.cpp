#include "ros/ros.h"
//#include "swerve_motion_profiling/GeneratePathAndProf.h"
#include "swerve_motion_profiling/profiler.h"
#include <boost/assign.hpp>
#include <pluginlib/class_list_macros.h>


std::shared_ptr<swerve_profile::swerve_profiler> profile_gen;

//void return_profile(swerve_motion_profiling::GenerateProfAndPath::Request &req, swerve_motion_profiling::GenerateProfAndPath::res &req
int main()
{
	profile_gen = std::make_shared<swerve_profile::swerve_profiler>(1, 9, 3, 6, 7, .01, .001);	
	return 0;
}
