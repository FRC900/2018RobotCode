#pragma once

#include <iostream>
#include <list>
#include <array>
#include <vector>
#include <cmath>
//#include <math.h>
#include <ros/console.h>

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
	typedef boost::geometry::model::d2::point_xy<double> point_type;
	typedef std::vector<point_type> polygon_edges;
	typedef boost::geometry::model::polygon<point_type> polygon_type;
class arm_limits
{
	public:	

		arm_limits() {};

		arm_limits(double min_extension, double max_extension, double x_back, double arm_length, 
		polygon_type remove_zone_down, int circle_point_count)
		{
			saved_polygons_ =  arm_limitation_polygon(min_extension, max_extension, 
			x_back, arm_length, remove_zone_down, circle_point_count);
			
			arm_length_ = arm_length;
			//dis be contructor
			//TODO: make better
		}
		bool check_if_possible(point_type &cmd, bool &up_or_down, int check_type)
		{		
			
			if(check_type < 2)
			{
				if(up_or_down)
				{
					if(boost::geometry::within(cmd, saved_polygons_[0]))
					{
						ROS_WARN("success");
						return true;
					}
					else if(boost::geometry::within(cmd, saved_polygons_[1]))
					{
						ROS_WARN("up to down");
						up_or_down = false;
						return false;
					}
					else
					{
						ROS_WARN("FAIL");

						find_nearest_point(cmd, up_or_down, check_type);
						return false;
					}
				}
				else
				{

					if(boost::geometry::within(cmd, saved_polygons_[1]))
					{
						ROS_WARN("success");	
						return true;
					}
					else if(boost::geometry::within(cmd, saved_polygons_[0]))
					{
						ROS_WARN("down to up");
						up_or_down = true;
						return false;
					}
					else
					{
						ROS_WARN("FAIL");
						find_nearest_point(cmd, up_or_down, check_type);
						return false;
					}
				}
			}
			else
			{
				if(up_or_down)
				{
					if(boost::geometry::within(cmd, saved_polygons_[0]))
					{
						ROS_WARN("success");
						return true;
					}
					else
					{
						ROS_WARN("FAIL");

						find_nearest_point(cmd, up_or_down, check_type);
						return false;
					}
				}
				else
				{

					if(boost::geometry::within(cmd, saved_polygons_[1]))
					{
						ROS_WARN("success");	
						return true;
					}
					else
					{
						ROS_WARN("FAIL");
						find_nearest_point(cmd, up_or_down, check_type);
						return false;
					}
				}
			}
		}
		bool safe_cmd(point_type &cmd, bool &up_or_down, bool &cmd_works, point_type &cur_pos, 
		bool &cur_up_or_down)
		{
			//Note: uses heuristics only applicable to our robot

			ROS_INFO_STREAM("cmd base check. Cmd: " << boost::geometry::wkt(cmd) << " up/down :" << up_or_down);
			cmd_works = check_if_possible(cmd, up_or_down, 0);
			
			ROS_INFO_STREAM("cur_pos check");
			check_if_possible(cur_pos, cur_up_or_down, 0);
			ROS_INFO_STREAM(" Cmd: " << boost::geometry::wkt(cur_pos) << " up/down :" << cur_up_or_down);
			
			double isolated_pivot_y =  sin(acos(cmd.x()/arm_length_))*arm_length_
			*( up_or_down ? 1 : -1) - sin(acos(cur_pos.x()/arm_length_))*arm_length_
			*(cur_up_or_down ? 1 : -1) + cur_pos.y();
			//Could switch above to using circle func instead of trig func	
			point_type test_pivot_cmd(cmd.x(), isolated_pivot_y);
			
			double isolated_lift_delta_y = cmd.y() - isolated_pivot_y;

			ROS_INFO_STREAM("pivot check. Cmd: " << boost::geometry::wkt(test_pivot_cmd) << " up/down :" << up_or_down);
			
			if(!check_if_possible(test_pivot_cmd, up_or_down, 1))
			{
				cmd.x(test_pivot_cmd.x());
				cmd.y(isolated_lift_delta_y + test_pivot_cmd.y());
				return false;				
			}
			else
			{
				point_type test_lift_cmd(cur_pos.x(), cur_pos.y() + isolated_lift_delta_y);
				if(!check_if_possible(test_lift_cmd, cur_up_or_down, 2))
				{
					cmd.y(test_lift_cmd.y() - cur_pos.y() + isolated_pivot_y);
					return false;
				}
				else
				{
					return true;
				}
			} 
			
			return true;
		}
	private:
		double arm_length_;
		std::array<polygon_type, 2> saved_polygons_;
		void find_nearest_point(point_type &cmd, bool &up_or_down, int check_type)
		{
			cmd.x(.48);
			cmd.y(1.5);
			up_or_down = true;
			//Make this an actual function
			//If check_type is 1, we only rotate to get it in the zone
			//If check_type is 2, we only may alter y to get it in the zone
			return;			
		}	
		polygon_edges quarter_circle_gen(double delta_height, double midpoint_z, double midpoint_x,			   int point_count, polygon_edges &circle, bool flip)
		{
			point_count++;
			//(z-midpoint_z)^2+(x-midpoint_x)^2 = radius^2
			//x = midpoint_x + sqrt(radius^2 - (z-midpoint_z)^2)


			for(int i  = 1; i < point_count; i++) //Don't need beginning line or end line
			{
				circle.push_back(point_type(midpoint_x+sqrt(pow(delta_height, 2) - pow(i*delta_height/point_count, 2)), i*delta_height/point_count + midpoint_z));
			}
			if(flip)
			{
				std::reverse(circle.begin(), circle.end());
			}
			for(int i  = 1; i < point_count; i++) //Don't need beginning line or end line
			{
				ROS_WARN("CIRCLE");
				ROS_INFO_STREAM("x: " << midpoint_x+sqrt(pow(delta_height, 2) - pow(i*delta_height/point_count, 2)) << " y: " << i*delta_height/point_count + midpoint_z);
			}
			return circle;
		}
		std::array<polygon_type, 2> arm_limitation_polygon(double min_extension, 
		double max_extension, double x_back, double arm_length, polygon_type remove_zone_down, 
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
			
			back_line_down.push_back(point_type(x_back, max_extension - arm_length));
			back_line_down.push_back(point_type(x_back, min_extension - arm_length));
			back_line_up.push_back(point_type(x_back, max_extension + arm_length));
			back_line_up.push_back(point_type(x_back, min_extension + arm_length));
			front_line_down.push_back(point_type(x_back + arm_length, min_extension));
			front_line_down.push_back(point_type(x_back + arm_length, max_extension)); 	
			front_line_up.push_back(point_type(x_back + arm_length, min_extension));
			front_line_up.push_back(point_type(x_back + arm_length, max_extension)); 	
			
			ROS_INFO_STREAM("poly up: " << boost::geometry::wkt(back_line_up[0])<< boost::geometry::wkt(back_line_up[1])<< boost::geometry::wkt(front_line_up[0])<< boost::geometry::wkt(front_line_up[1]));
			
			quarter_circle_gen(arm_length, max_extension, x_back, circle_point_count, top_circle_up, false);
			quarter_circle_gen(-arm_length, max_extension, x_back, circle_point_count, top_circle_down, false);
			quarter_circle_gen(arm_length, min_extension, x_back, circle_point_count, bottom_circle_up, true);
			quarter_circle_gen(-arm_length, min_extension, x_back, circle_point_count, bottom_circle_down, true);
	
			for(int i = 0; i < top_circle_up.size(); i++)
			{
				ROS_INFO_STREAM("up poly circle up: " << boost::geometry::wkt(top_circle_up[i]));

			}
			for(int i = 0; i < bottom_circle_up.size(); i++)
			{
				ROS_INFO_STREAM("up poly circle bottom: " << boost::geometry::wkt(bottom_circle_up[i]));

			}
			ROS_WARN("showing up poly circles");
			//for(int i = 0; i < top_circle_up.size()	
			//insert into back line

			for(int i = 0; i < back_line_up.size(); i++)
			{
				ROS_INFO_STREAM("up poly point just back: " << boost::geometry::wkt(back_line_up[i]));

			}

			back_line_up.insert(back_line_up.end(), bottom_circle_up.begin(), bottom_circle_up.end());
			for(int i = 0; i < back_line_up.size(); i++)
			{
				ROS_INFO_STREAM("up poly point just back + bottom circle: " << boost::geometry::wkt(back_line_up[i]));

			}
			back_line_up.insert(back_line_up.end(), front_line_up.begin(), front_line_up.end());
			for(int i = 0; i < back_line_up.size(); i++)
			{
				ROS_INFO_STREAM("up poly point just back + bottom circle + front: " << boost::geometry::wkt(back_line_up[i]));

			}
			back_line_up.insert(back_line_up.end(), top_circle_up.begin(), top_circle_up.end());
			back_line_up.push_back(back_line_up[0]);


			back_line_down.insert(back_line_down.end(), bottom_circle_down.begin(), bottom_circle_down.end());
			back_line_down.insert(back_line_down.end(), front_line_down.begin(), front_line_down.end());
			back_line_down.insert(back_line_down.end(), top_circle_down.begin(), top_circle_down.end());
			back_line_down.push_back(back_line_down[0]);

			polygon_type down_poly;
			polygon_type up_poly;
			
			
			for(int i = 0; i < back_line_up.size(); i++)
			{
				ROS_INFO_STREAM("up poly point: " << boost::geometry::wkt(back_line_up[i]));

			}
	
			boost::geometry::assign_points(down_poly, back_line_down);
			boost::geometry::assign_points(up_poly, back_line_up);

			ROS_INFO_STREAM("poly up: " << boost::geometry::wkt(up_poly));
			ROS_INFO_STREAM("poly down: " << boost::geometry::wkt(down_poly));
			//TODO: remove excluded points from poly
			//Also, test these polys with print statements
			
			std::array<polygon_type, 2> up_and_down_polygons = {up_poly, down_poly};
			return up_and_down_polygons;
		}
};
}

