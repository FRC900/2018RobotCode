#include <base_trajectory/profiler.h>
#include <ros/console.h>

namespace swerve_profile
{
swerve_profiler::swerve_profiler(double max_wheel_dist, double max_wheel_mid_accel, double max_wheel_vel,
double max_steering_accel, double max_steering_vel, double dt)
{
	max_wheel_dist_ = max_wheel_dist;
	max_wheel_mid_accel_ = max_wheel_mid_accel;
	max_wheel_vel_ = max_wheel_vel;
	max_steering_accel_ = max_steering_accel;
	max_steering_vel_ = max_steering_vel;
	dt_ = dt;
}
//TODO :: path should be const vect & to avoid a redundant copy
// being made each time the function is called
bool swerve_profiler::generate_profile(const std::vector<spline_coefs> &x_splines, const std::vector<spline_coefs> &y_splines, const std::vector<spline_coefs> &orient_splines, const double &initial_v, const double &final_v, base_trajectory::GenerateSwerveProfile::Response &out_msg, const std::vector<double> &end_points)
{
	tk::spline spline;
	double total_arc;

	

	double curr_v = initial_v;
	std::vector<double> velocities;
	velocities.reserve(155 / dt_); //For full auto :) 
	std::vector<double> positions;
	positions.reserve(155 / dt_); //For full auto :) 

	path_point holder_point;
	double t;
	double dtds;
	spline_coefs holder_spline;

	std::vector<spline_coefs> x_splines_first_deriv, y_splines_first_deriv, orient_splines_first_deriv ;
	std::vector<spline_coefs> x_splines_second_deriv, y_splines_second_deriv, orient_splines_second_deriv ;
	for(size_t i = 0; i < x_splines.size(); i++)
	{
		holder_spline.a = 0;
		holder_spline.b = 5*x_splines[i].a;
		holder_spline.c = 4*x_splines[i].b;
		holder_spline.d = 3*x_splines[i].c;
		holder_spline.e = 2*x_splines[i].d;
		holder_spline.f = 1*x_splines[i].e;
	
		x_splines_first_deriv.push_back(holder_spline);
		
		
		holder_spline.f = 1*holder_spline.e;
		holder_spline.e = 2*holder_spline.d;
		holder_spline.d = 3*holder_spline.c;
		holder_spline.c = 4*holder_spline.b;
		holder_spline.b = 0;
		holder_spline.a = 0;
	
		x_splines_second_deriv.push_back(holder_spline);

		holder_spline.a = 0;
		holder_spline.b = 5*y_splines[i].a;
		holder_spline.c = 4*y_splines[i].b;
		holder_spline.d = 3*y_splines[i].c;
		holder_spline.e = 2*y_splines[i].d;
		holder_spline.f = 1*y_splines[i].e;
	
		y_splines_first_deriv.push_back(holder_spline);
		
		
		holder_spline.f = 1*holder_spline.e;
		holder_spline.e = 2*holder_spline.d;
		holder_spline.d = 3*holder_spline.c;
		holder_spline.c = 4*holder_spline.b;
		holder_spline.b = 0;
		holder_spline.a = 0;
	
		y_splines_second_deriv.push_back(holder_spline);

		holder_spline.a = 0;
		holder_spline.b = 5*orient_splines[i].a;
		holder_spline.c = 4*orient_splines[i].b;
		holder_spline.d = 3*orient_splines[i].c;
		holder_spline.e = 2*orient_splines[i].d;
		holder_spline.f = 1*orient_splines[i].e;
	
		orient_splines_first_deriv.push_back(holder_spline);
		
		
		holder_spline.f = 1*holder_spline.e;
		holder_spline.e = 2*holder_spline.d;
		holder_spline.d = 3*holder_spline.c;
		holder_spline.c = 4*holder_spline.b;
		holder_spline.b = 0;
		holder_spline.a = 0;
	
		orient_splines_second_deriv.push_back(holder_spline);
	}
	
	spline = parametrize_spline(x_splines_first_deriv, y_splines_first_deriv, end_points, total_arc);
	//back pass
	
	for(double i = total_arc; i > 0;)
	{
		i -= curr_v*dt_;
		
		velocities.push_back(curr_v);
		positions.push_back(i);
		
		t = spline(i);
		
		dtds = (spline(i+.001) - spline(i-.001)) / .002; //Maybe change spline and analytically deriv?
	
		comp_point_characteristics(x_splines, y_splines, x_splines_first_deriv, y_splines_first_deriv, x_splines_second_deriv, y_splines_second_deriv, orient_splines, orient_splines_first_deriv, orient_splines_second_deriv, t, holder_point, end_points, dtds);
		

		if(!solve_for_next_V(holder_point, total_arc, curr_v, i))
		{
			return false;
		}			
	}
	out_msg.points.resize(155 / dt_); //For full auto :)  TODO: optimize
	curr_v = final_v;
	double starting_point = 0;
	double vel_cap;
	int point_count = 0;
	ros::Duration now(0);
	ros::Duration period(dt_);
	for(double i = 0; i < total_arc;)
	{
		i += curr_v*dt_;	
		
		t = spline(i);

		dtds = (spline(i+.001) - spline(i-.001)) / .002; //Maybe change spline and analytically deriv?
		
		comp_point_characteristics(x_splines, y_splines, x_splines_first_deriv, y_splines_first_deriv, x_splines_second_deriv, y_splines_second_deriv, orient_splines, orient_splines_first_deriv, orient_splines_second_deriv, t, holder_point, end_points, dtds);
	
		//TODO: CHECK CONVERSIONS

		ROS_INFO_STREAM("t: " << t << " pos: " << holder_point.pos);
	
		out_msg.points[point_count].positions.push_back(holder_point.pos[0]);
		out_msg.points[point_count].positions.push_back(holder_point.pos[1]);
		out_msg.points[point_count].positions.push_back(holder_point.orientation);
		out_msg.points[point_count].velocities.push_back(cos(holder_point.path_angle) * curr_v );
		out_msg.points[point_count].velocities.push_back(sin(holder_point.path_angle) * curr_v );
		out_msg.points[point_count].velocities.push_back(holder_point.angular_velocity * max_wheel_dist_* (curr_v / (total_arc)));
		//out_msg.points[point_count].velocities.push_back(holder_point.path_angle_deriv * (current_v));
		out_msg.points[point_count].time_from_start = now;
		now += period;
		point_count++;
		if(!solve_for_next_V(holder_point, total_arc, curr_v, i))
		{
			return false;
		}			
		for(size_t k = 0; k < positions.size() - starting_point; k++)
		{
			if(positions[starting_point+k] > i)
			{
				starting_point += k;
				break;
			}
			//Find point
		}
		//Linear interpolation
		vel_cap = i * (velocities[starting_point] - velocities[starting_point - 1]) / 
		(positions[starting_point] - positions[starting_point - 1]) - positions[starting_point] *
		(velocities[starting_point] - velocities[starting_point - 1]) / 
		(positions[starting_point] - positions[starting_point - 1]) + velocities[starting_point];
		//Keep below back pass	
		coerce(curr_v, -100000000000, vel_cap);
	}
	out_msg.points.erase(out_msg.points.begin() + point_count, out_msg.points.end());	
	return true;
}
// TODO :: is return code needed here?
bool swerve_profiler::coerce(double &val, const double &min, const double &max)
{
	if(val > max)
	{
		val = max;
		return true;
	}
	else if(val < min)
	{
		val = min;
		return true;
	}
	else
	{
		return false;
	}	
}
bool swerve_profiler::solve_for_next_V(const path_point &path, const double &path_length, double &current_v, const double &current_pos)
{
	// TODO - double-check that these need to be static
	//TODO: CHECK CONVERSIONS
	static double v_general_max;
	static double v_curve_max; 
	static double eff_max_a;
	static double max_wheel_orientation_vel;
	static double max_wheel_orientation_accel;
	static double accel;
	static double theta;
	static double cos_t;
	static double sin_t;
	static double path_induced_a;
	if(current_pos<=0 && current_pos>=path_length - 1)
	{
		max_wheel_orientation_accel = path.angular_accel * (current_v )* (current_v );
		max_wheel_orientation_vel = path.angular_velocity * (current_v );
		theta = fabs(fmod(path.path_angle - path.orientation, M_PI / 8));
		cos_t = cos(theta);
		sin_t = sin(theta);
		path_induced_a = current_v*current_v/path.radius;
		
		coerce(eff_max_a, 0, max_wheel_mid_accel_); //Consider disabling this coerce
		
		// TODO : check return code here
		// TODO : Use explicit multiply rather than pow() for squaring stuff
		poly_solve(1, 4*cos_t*sin_t*path_induced_a + sqrt(2)*cos_t*max_wheel_orientation_accel + sqrt(2)*sin_t*max_wheel_orientation_accel, path_induced_a*path_induced_a + sqrt(2)*sin_t*path_induced_a*max_wheel_orientation_accel + sqrt(2)*cos_t*path_induced_a*max_wheel_orientation_accel + max_wheel_orientation_accel*max_wheel_orientation_accel - max_wheel_mid_accel_ * max_wheel_mid_accel_, accel);

		current_v += accel * dt_;
		if(!poly_solve(1, sqrt(2) *  max_wheel_orientation_vel * cos_t + sqrt(2) *  max_wheel_orientation_vel * sin_t, max_wheel_orientation_vel * max_wheel_orientation_vel- max_wheel_vel_ * max_wheel_vel_, v_general_max))
			return false;
		//Note: assumption is that angular velocity doesn't change much over timestep
		coerce(current_v, -v_general_max, v_general_max); 
		//consider using above coerce in a if statement for optimization
		
		eff_max_a = max_wheel_mid_accel_ * 2 * (1 -  (max_wheel_vel_ - sqrt((current_v + max_wheel_orientation_vel * sqrt(2)/2) * (current_v + max_wheel_orientation_vel * sqrt(2)/2) + max_wheel_orientation_vel * max_wheel_orientation_vel / 2)) / max_wheel_vel_);

		v_curve_max = sqrt(eff_max_a * eff_max_a/ (1/(path.radius * path.radius) + sqrt(2) * sin_t * path.angular_accel/path.radius + sqrt(2) * cos_t * path.angular_accel/path.radius + path.angular_accel * path.angular_accel));
		coerce(current_v, -v_curve_max, v_curve_max);
	}
	else
	{	
		current_v += max_wheel_mid_accel_;
		coerce(current_v, -max_wheel_vel_, max_wheel_vel_);	
		return true;
	}
}
tk::spline swerve_profiler::parametrize_spline(const std::vector<spline_coefs> &x_splines_first_deriv, const std::vector<spline_coefs> &y_splines_first_deriv, std::vector<double> end_points, double &total_arc_length)
{	
	total_arc_length = 0;
	double period_t = end_points[0] - 0;
	double start = 0;
	double a_val;
	double b_val;
	double x_at_a;
	double x_at_b;
	double y_at_a;
	double y_at_b;
	double x_at_avg;
	double y_at_avg;
	std::vector<double> t_vals;
	std::vector<double> s_vals;
	t_vals.reserve(x_splines_first_deriv.size() * 101);
	s_vals.reserve(x_splines_first_deriv.size() * 101);
	ROS_INFO_STREAM("Running parametrize");
	//TODO: Make i not start at 1
	for(size_t i = 1; i < x_splines_first_deriv.size(); i++)
	{
		if(i != 0)
		{
			period_t = (end_points[i] - end_points[i-1])/100.0; //100 is super arbitrary
			start = end_points[i-1];
		}
		for(size_t k = 0; k < 100; k++)
		{
			a_val = k*period_t + start;
			b_val = (k+1)*period_t + start;
			t_vals.push_back(a_val);
			s_vals.push_back(total_arc_length);
			calc_point(x_splines_first_deriv[i], a_val, x_at_a);
			calc_point(x_splines_first_deriv[i], b_val, x_at_b);
			calc_point(y_splines_first_deriv[i], a_val, y_at_a);
			calc_point(y_splines_first_deriv[i], b_val, y_at_b);
			calc_point(x_splines_first_deriv[i], (a_val+b_val)/2, x_at_avg);
			calc_point(y_splines_first_deriv[i], (a_val+b_val)/2, y_at_avg);
		
			//f(t) = sqrt((dx/dt)^2 + (dy/dt)^2)

			total_arc_length += period_t / 6 * (sqrt(x_at_a*x_at_a + y_at_a*y_at_a) + 4 * 
			sqrt(x_at_avg* x_at_avg+ y_at_avg* y_at_avg) + sqrt(x_at_b*x_at_b + y_at_b*y_at_b));
			ROS_INFO_STREAM("Spline: " << i << " t_val: " << a_val <<"  arc_length: " << total_arc_length);
		} 
	}
	t_vals.push_back(b_val);
	s_vals.push_back(total_arc_length);
	//TODO: Loop to generate set of s vals for t vals iterating from 0 to end_time (simpsons rule here)

	
	//Spline fit of t interms of s (we input a t -> s)
	tk::spline s;
	s.set_points(s_vals, t_vals);
	ROS_INFO_STREAM("Spline_test: " << s(.5) << "  arc_length: " << total_arc_length);
	
	return s;
}
bool swerve_profiler::poly_solve(const double &a, const double &b, const double &c, double &x)
{
	const double det = b*b - 4 * a * c;
	if(det < 0)
	{
		return false;
	}
	else
	{
		x = (-b + sqrt(det))/(2*a); //This is just one of the roots, but it should be fine for all 
					    //cases it is used here
		return true;
	}
}
void swerve_profiler::calc_point(const spline_coefs &spline, const double t, double &returner)
{
	returner = spline.a * t*t*t*t*t + spline.b * t*t*t*t + spline.c * t*t*t + spline.d * t*t + spline.e * t + spline.f; 
}
void swerve_profiler::comp_point_characteristics(const std::vector<spline_coefs> &x_splines, const std::vector<spline_coefs> &y_splines, const std::vector<spline_coefs> &x_splines_first_deriv, const std::vector<spline_coefs> &y_splines_first_deriv, const std::vector<spline_coefs> &x_splines_second_deriv, const std::vector<spline_coefs> &y_splines_second_deriv, const std::vector<spline_coefs> &orient_splines, const std::vector<spline_coefs> &orient_splines_first_deriv, const std::vector<spline_coefs> &orient_splines_second_deriv, double t, path_point &holder_point, const std::vector<double> &end_points, const double &dtds )
{
	static int which_spline;
	which_spline = 0;
	for(;which_spline < x_splines.size(); which_spline++)
	{
		if(t < end_points[which_spline])
		{
			break;
		}
	}
	static double orient;
	static double first_deriv_orient;
	static double second_deriv_orient;
	static double point_x;
	static double point_y;
	static double first_deriv_x;
	static double first_deriv_y;
	static double second_deriv_x;
	static double second_deriv_y;

	
	calc_point(x_splines[which_spline], t, point_x);
	calc_point(y_splines[which_spline], t, point_y);
	calc_point(x_splines_first_deriv[which_spline], t, first_deriv_x);
	calc_point(y_splines_first_deriv[which_spline], t, first_deriv_y);
	calc_point(x_splines_second_deriv[which_spline], t, second_deriv_x);
	calc_point(y_splines_second_deriv[which_spline], t, second_deriv_y);
	calc_point(orient_splines[which_spline], t, orient);
	calc_point(orient_splines_first_deriv[which_spline], t, first_deriv_orient);
	calc_point(orient_splines_second_deriv[which_spline], t, second_deriv_orient);
	
	ROS_INFO_STREAM("which spline: " << which_spline << " x: " << point_x << " y: " << point_y << " a: " << x_splines[which_spline].a <<" b: " << x_splines[which_spline].b <<" c: " << x_splines[which_spline].c <<" d: " << x_splines[which_spline].d <<" e: " << x_splines[which_spline].e <<" f: " << x_splines[which_spline].f);
	
	//Radius = (x'^2 + y'^2)^(3/2) / (x' * y'' - y' * x'')

	holder_point.radius = fabs(pow(first_deriv_x*first_deriv_x + first_deriv_y*first_deriv_y, 3/2) / 
	(first_deriv_x * second_deriv_y - first_deriv_y*second_deriv_x));

	holder_point.pos[0] = point_x;
	holder_point.pos[1] = point_y;

	holder_point.path_angle = atan2(first_deriv_y, first_deriv_x);  //Make sure this is what we want

	holder_point.orientation = orient;	
	
	holder_point.angular_velocity = first_deriv_orient * dtds * max_wheel_dist_;	

	holder_point.angular_accel = second_deriv_orient * dtds * dtds * max_wheel_dist_;	
}
}
