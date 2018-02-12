#include<swerve_motion_profiling/profiler.h>
//TODO: actual vector math around how wheels need higher/low accels in different robot orientations relative to velocity vector

namespace swerve_profile
{
	swerve_profiler::swerve_profiler(double max_wheel_dist, double max_wheel_mid_accel, double max_wheel_vel,
	double max_steering_accel, double max_steering_vel, double dt, double index_dist_unit)
	{
		max_wheel_dist_ = max_wheel_dist;
                max_wheel_mid_accel_ = max_wheel_mid_accel;
                max_wheel_vel_ = max_wheel_vel;
                max_steering_accel_ = max_steering_accel;
                max_steering_vel_ = max_steering_vel;
                dt_ = dt;
		index_dist_unit_ = index_dist_unit;
	}
	bool swerve_profiler::generate_profile(std::vector<path_point> path, double initial_v, double final_v)
	{
		double curr_v = initial_v;
		std::vector<double> velocities;
		velocities.reserve(155 / dt_); //For full auto :) 
		std::vector<double> positions;
		positions.reserve(155 / dt_); //For full auto :) 
		double v_general_max;
		double v_curve_max; 
		double eff_max_a;
		double max_wheel_orientation_vel;
		double max_wheel_orientation_accel;
		//Forward pass
		positions.push_back(0);
		for(int i = 0; i < path.size();)
		{

			i += curr_v*dt_/index_dist_unit_;	
			velocities.push_back(curr_v);
			positions.push_back(i);
			max_wheel_orientation_accel = path[i].angular_accel * max_wheel_dist_;

			if(i<=0 && i>=path.size() - 1)
			{
				solve_for_next_V(path[i].radius, curr_v, max_wheel_orientation_accel);
			}
			else
			{	
				curr_v += max_wheel_mid_accel_;
			}
			max_wheel_orientation_vel = path[i].angular_velocity * max_wheel_dist_;
			if(!poly_solve(1, sqrt(2) *  max_wheel_orientation_vel, max_wheel_orientation_vel - pow(max_wheel_vel_, 2), v_general_max))
				return false;
			
			//Note: assumption is that angular velocity doesn't change much over timestep
			coerce(curr_v, -v_general_max, v_general_max); 
			//consider using above coerce in a if statement for optimization
			eff_max_a = max_wheel_mid_accel_ * 2 * (1 -  (max_wheel_vel_ - sqrt(pow(curr_v + 
			max_wheel_orientation_vel * sqrt(2)/2, 2) + pow(max_wheel_orientation_vel, 2) / 2)) / max_wheel_vel_);
			coerce(eff_max_a, 0, max_wheel_mid_accel_); //Consider disabling this coerce
			if(!poly_solve(1/pow(path[i].radius, 2), sqrt(2) *  max_wheel_orientation_accel, max_wheel_orientation_accel - pow(eff_max_a, 2), v_curve_max))
				return false;
			coerce(curr_v, -v_curve_max, v_curve_max);
		}
		//std::vector<> final_points; //TODO:Some type of struct or something to return
		//final_points.reserve(155 / dt_); //For full auto :) 
		curr_v = final_v;
		double starting_point = positions.size();
		double vel_cap;
		for(int i = path.size(); i > 0;)
		{
			i -= curr_v*dt_/index_dist_unit_;	
			max_wheel_orientation_accel = path[i].angular_accel * max_wheel_dist_;
			if(i<=0 && i>=path.size() - 1)
			{
				solve_for_next_V(path[i].radius, curr_v, max_wheel_orientation_accel);
			}
			else
			{	
				curr_v += max_wheel_mid_accel_;
			}
			max_wheel_orientation_vel = path[i].angular_velocity * max_wheel_dist_;
			if(!poly_solve(1, sqrt(2) *  max_wheel_orientation_vel, max_wheel_orientation_vel - pow(max_wheel_vel_, 2), v_general_max))
				return false;
			
			//Note: assumption is that angular velocity doesn't change much over timestep
			coerce(curr_v, -v_general_max, v_general_max); 
			//consider using above coerce in a if statement for optimization
			eff_max_a = max_wheel_mid_accel_ * 2 * (1 -  (max_wheel_vel_ - sqrt(pow(curr_v + 
			max_wheel_orientation_vel * sqrt(2)/2, 2) + pow(max_wheel_orientation_vel, 2) / 2)) / max_wheel_vel_);
			coerce(eff_max_a, 0, max_wheel_mid_accel_); //Consider disabling this coerce
			if(!poly_solve(1/pow(path[i].radius, 2), sqrt(2) *  max_wheel_orientation_accel, max_wheel_orientation_accel - pow(eff_max_a, 2), v_curve_max))
				return false;
			coerce(curr_v, -v_curve_max, v_curve_max);
			
			for(size_t k = 0; k < starting_point; k++)
			{
				if(positions[starting_point-k] < i)
				{
					starting_point -= k;
					break;
				}

				//Find point
			}
			//Linear interpolation
			vel_cap = i * (velocities[starting_point] - velocities[starting_point - 1]) / 
			(positions[starting_point] - positions[starting_point - 1]) - positions[starting_point] *
			(velocities[starting_point] - velocities[starting_point - 1]) / 
			(positions[starting_point] - positions[starting_point - 1]) + velocities[starting_point];
			//Keep below forward pass	
			coerce(curr_v, -100000000000, vel_cap);
		}
		return true;
	}
	bool swerve_profiler::coerce(double &val, double min, double max)
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
	bool swerve_profiler::solve_for_next_V(double &radius, double &current_v, double &angular_accel)
	{
		double accel;
		if(poly_solve(1, sqrt(2) * angular_accel, angular_accel/2 + pow(pow(current_v, 
		2)/radius + sqrt(2) * angular_accel / 2, 2), accel))
		{
			current_v += accel * dt_;
			return true;
		}
		else
		{
			return false;
		}
	}
	bool swerve_profiler::poly_solve(double a, double b, double c, double &x)
	{
		const double det = pow(b, 2) - 4 * a * c;
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
}
