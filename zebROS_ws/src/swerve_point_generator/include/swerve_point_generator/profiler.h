#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include <swerve_point_generator/GenerateSwerveProfile.h>
#include <swerve_point_generator/spline.h>
//#include <ros/ros.h>

namespace swerve_profile
{

//Indices correspond to distances rather than including that in the struct

struct path_point
{
	double radius;
	Eigen::Vector2d pos;
	double path_angle;
	//double path_angle_deriv;
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
struct spline_coefs
{
	double a;
	double b;
	double c;
	double d;
	double e;	
	double f;
	spline_coefs(void):
		a(0),
		b(0),
		c(0),
		d(0),
		e(0),
		f(0)
	{
	}
};
class swerve_profiler
{
	public:
		swerve_profiler(double max_wheel_dist, double max_wheel_mid_accel, double max_wheel_vel, 
		double max_steering_accel, double max_steering_vel, double dt, double ang_accel_conv, double max_wheel_brake_accel);
			
		bool generate_profile(std::vector<spline_coefs> x_splines, std::vector<spline_coefs> y_splines, std::vector<spline_coefs> orient_splines, const double initial_v, const double final_v, swerve_point_generator::GenerateSwerveProfile::Response &out_msg, const std::vector<double> &end_points, double t_shift, bool flip_dirc);

		//TODO: maybe add options to above functions?
			
	private:
		void comp_point_characteristics(const std::vector<spline_coefs> &x_splines, const std::vector<spline_coefs> &y_splines, const std::vector<spline_coefs> &x_splines_first_deriv, const std::vector<spline_coefs> &y_splines_first_deriv, const std::vector<spline_coefs> &x_splines_second_deriv, const std::vector<spline_coefs> &y_splines_second_deriv, const std::vector<spline_coefs> &orient_splines, const std::vector<spline_coefs> &orient_splines_first_deriv, const std::vector<spline_coefs> &orient_splines_second_deriv, path_point &holder_point, const std::vector<double> &end_points, const std::vector<double> &dtds_by_spline, const double raw_t);

		tk::spline parametrize_spline(const std::vector<spline_coefs> &x_spline, const std::vector<spline_coefs> &y_spline, std::vector<double> end_points, double &total_arc_length, std::vector<double> &dtds_by_spline);
		void calc_point(const spline_coefs &spline, const double t, double &returner);
		bool solve_for_next_V(const path_point &path, const double path_length, double &current_v, const double current_pos, double &accel_defined);  //most pointers are just for efficiency
		bool coerce(double &val, const double min, const double max); 
		bool poly_solve(const double a, const double b, const double c, double &x);
		double max_wheel_dist_;
		double max_wheel_mid_accel_;
		double max_wheel_vel_;
		double max_steering_accel_;
		double max_steering_vel_;
		double dt_;
		double t_shift_;
		bool flip_dirc_;
		double t_total_;
		double ang_accel_conv_;
		bool fow;
		double max_wheel_brake_accel_;
};
}
