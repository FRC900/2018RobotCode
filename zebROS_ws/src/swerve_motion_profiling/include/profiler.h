#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <vector>
//#include <ros/ros.h>

namespace swerve_profile
{

//Indices correspond to distances rather than including that in the struct

struct path_point
{
	double radius;
	Eigen::Vector2d pos;
	double orientation;
	double angular_velocity;
	double angular_accel;
	path_point(void):
		radius(1000000000000000000000000),
		pos({0,0}),
		orientation(0),
		angular_velocity(0),
		angular_accel(0)
	{
	}
};
class swerve_profiler
{
	public:
		swerve_profiler(double max_wheel_dist, double max_wheel_mid_accel, double max_wheel_vel, 
		double max_steering_accel, double max_steering_vel, double dt, double index_dist_unit);

		bool generate_profile(std::vector<path_point> path, double initial_v, double final_v);
		//TODO: maybe add options to above functions?
	
	private:
		bool solve_for_next_V(double &radius, double &current_v, double &angular_accel);
		bool coerce(double &val, double &min, double &max); //most pointers are just for efficiency
		bool poly_solve(double a, double b, double c, double &x);
		double max_wheel_dist_;
		double max_wheel_mid_accel_;
		double max_wheel_vel_;
		double max_steering_accel_;
		double max_steering_vel_;
		double dt_;
		double index_dist_unit_;
};
}
