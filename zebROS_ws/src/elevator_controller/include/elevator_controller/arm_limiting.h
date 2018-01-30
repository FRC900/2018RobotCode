#pragma once

#include <iostream>
#include <list>
#include <array>
#include <vector>
#include <cmath>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <iostream>
#include <vector>
#include <boost/assign/std/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/dsv/write.hpp>


namespace arm_limiting
{
	typedef std::vector<point_type> polygon_edges;
	typedef boost::geometry::model::d2::point_xy<double> point_type;
	typedef boost::geometry::model::polygon<point_type> polygon_type;
class arm_limits
{
	public:	



		arm_limits(double min_extension, double max_extension, double x_back, double arm_length, 
		polygon_edges remove_zone_down, int circle_point_count)
		{
			saved_polygons_ =  arm_limitation_polygon(min_extension, max_extension, 
			x_back, arm_length, remove_zone_down, circle_point_count);
			
			arm_length_ = arm_length;
			//dis be contructor
			//TODO: make better
		}
		bool check_if_possible(point_type &cmd, bool &up_or_down)
		{		
			if(up_or_down)
			{
				if(boost::geometry::within(cmd, saved_polygons_[0])
				{
					return true;
				}
				else if(boost::geometry::within(cmd, saved_polygons_[1])
				{
					up_or_down = false;
					return false;
				}
				else
				{
					find_nearest_point(cmd, up_or_down);
					return false;
				}
			}
			else
			{

				if(boost::geometry::within(cmd, saved_polygons_[1])
				{
					return true;
				}
				else if(boost::geometry::within(cmd, saved_polygons_[0])
				{
					up_or_down = true;
					return false;
				}
				else
				{
					find_nearest_point(cmd, up_or_down);
					return false;
				}
			}
		}
		bool safe_cmd(point_type &cmd, bool &up_or_down, bool &reassigned, point_type cur_pos, 
		bool cur_up_or_down)
		{
			//Note: uses heuristics only applicable to our robot
			reassigned = check_if_possible(cmd, up_or_down);
			
			double isolated_pivot_y =  sin(acos(cmd.x()/arm_length_))*arm_length_
			*((up_or_down?) 1 : -1) - sin(acos(cur_pos.x()/arm_length_))*arm_length_
			*((cur_up_or_down?) 1 : -1) + cur_pos.y();
			//Could switch above to using circle func instead of trig func	
			point_type test_pivot_cmd(cmd.x(), isolated_pivot_y);
			
			double isolated_lift_delta_y = cmd.y() - isolated_pivot_y;

			if(!check_if_possible(test_pivot_cmd, up_or_down))
			{
				cmd.x(test_pivot_cmd.x());
				cmd.y(test_pivot_cmd.y(isolated_lift_delta_y+test_pivot_cmd.y());
				return false;				
			}
			else
			{
				point_type test_lift_cmd(cur_pos.x(), cur_pos.y() + isolated_lift_delta_y);
				bool temp = up_or_down;
				if(!check_if_possible(test_lift_cmd, temp))
				{
					cmd.y(test_lift_cmd.y() - cur_pos.y() + isolated_pivot_y);
					return false;
				}
				else
				{
					return true;
				}
			} 
		}
	private:
		double arm_length_;
		std::array<polygon_type, 2> saved_polygons_;
		find_nearest_point(point_type &cmd, bool &up_or_down)
		{
			



		}	
		polygon_edges quarter_circle_gen(double delta_height, double midpoint_z, double midpoint_x,			   int point_count)
		{
			polygon_edges circle;
			circle.resize(point_count);
			point_count++;
			//(z-midpoint_z)^2+(x-midpoint_x)^2 = radius^2
			//x = midpoint_x + sqrt(radius^2 - (z-midpoint_z)^2)
			for(int i  = 1, i < point_count, i++) //Don't need beginning line or end line
			{
				circle += point_type(midpoint_x+sqrt(pow(delta_height, 2) - pow(i*delta_height/point_count, 2)), i*delta_height/point_count + midpoint_z);
			}
			if(delta_height < 0)
			{
				std::reverse(circle.begin(), circle.end());
			}
			return circle;
		}
		std::array<polygon_type, 2> arm_limitation_polygon(double min_extension, 
		double max_extension, double x_back, double arm_length, polygon_edges remove_zone_down, 
		int circle_point_count)
		{
			polygon_edges back_line_down;
			polygon_edges back_line_up;
			polygon_edges front_line_up;
			polygon_edges front_line_down;

			polygon_edges top_circle_down;
			polygon_edges top_circle_up;
			polygon_edges bottom_circle_down;
			polygon_edges bottom_circle_up;

			back_line_down += point_type(x_back, max_extension - arm_length), point_type(x_back, min_extension - arm_length);
			back_line_up +=  point_type(x_back, max_extension + arm_length), point_type(x_back, min_extension + arm_length);
			front_line_down += point_type(x_back + arm_length, min_extension), point_type(x_back + arm_length, max_extension); 	
			front_line_up += point_type(x_back + arm_length, min_extension), point_type(x_back + arm_length, max_extension); 	
			
			top_circle_up = quarter_circle_gen(arm_length, max_extension, x_back, circle_point_count);
			top_circle_down = quarter_circle_gen(-arm_length, max_extension, x_back, circle_point_count);
			bottom_circle_up = quarter_circle_gen(arm_length, min_extension, x_back, circle_point_count);
			bottom_circle_down = quarter_circle_gen(-arm_length, min_extension, x_back, circle_point_count);
		
			//insert into back line


			back_line_up.insert(back_line_up.end(), bottom_circle_up.begin(), bottom_circle_up.end());
			back_line_up.insert(back_line_up.end(), front_line_up.begin(), front_line_up.end());
			back_line_up.insert(back_line_up.end(), top_circle_up.begin(), top_circle_up.end());
			back_line_up.push_back(back_line_up[0]);


			back_line_down.insert(back_line_down.end(), bottom_circle_down.begin(), bottom_circle_down.end());
			back_line_down.insert(back_line_down.end(), front_line_down.begin(), front_line_down.end());
			back_line_down.insert(back_line_down.end(), top_circle_down.begin(), top_circle_down.end());
			back_line_down.push_back(back_line_down[0]);

			polygon down_poly;
			polygon up_poly;
			
			boost::geometry::assign_points(down_poly, back_line_down);
			boost::geometry::assign_points(up_poly, back_line_up);

			//TODO: remove excluded points from poly
			//Also, test these polys with print statements
			
			std::array<polygon_type, 2> up_and_down = {up_poly, down_poly};
			return up_and_down;
		}
};
}

