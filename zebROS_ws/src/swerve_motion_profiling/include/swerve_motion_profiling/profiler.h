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
	double path_angle;
	double orientation;
	double angular_velocity;
	double angular_accel;
	path_point(void):
		radius(1000000000000000000.0),
		pos({0,0}),
		path_angle(0),
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

		bool generate_profile(const std::vector<path_point> &path, const double &initial_v, const double &final_v);
		//TODO: maybe add options to above functions?
	
	private:
		bool solve_for_next_V(const double &i, const std::vector<path_point> &path, double &current_v);//most pointers are just for efficiency
		bool coerce(double &val, const double &min, const double &max); 
		bool poly_solve(const double &a, const double &b, const double &c, double &x);
		double max_wheel_dist_;
		double max_wheel_mid_accel_;
		double max_wheel_vel_;
		double max_steering_accel_;
		double max_steering_vel_;
		double dt_;
		double index_dist_unit_;
};
}
