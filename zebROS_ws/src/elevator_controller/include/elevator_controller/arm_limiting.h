#pragma once

#include <iostream>
#include <list>
#include <array>
#include <vector>
#include <cmath>
//#include <math.h>
#include <ros/console.h>

#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <boost/assign/std/vector.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/dsv/write.hpp>

using std::vector;
namespace arm_limiting
{

	typedef boost::geometry::model::d2::point_xy<double> point_type;
	typedef std::vector<point_type> polygon_edges;
	typedef boost::geometry::model::polygon<point_type> polygon_type;
	typedef boost::geometry::model::linestring<point_type> linestring_type;
class arm_limits
{
	public:	

		arm_limits() {};

		arm_limits(double min_extension, double max_extension, double x_back, double arm_length, 
		polygon_type remove_zone_down, int circle_point_count)
		{
			saved_polygons_ =  arm_limitation_polygon(min_extension, max_extension, 
			x_back, arm_length, remove_zone_down, circle_point_count);
		
			max_extension_ = max_extension;
			min_extension_ = min_extension;
				
			arm_length_ = arm_length;
			point_type top(-.01, arm_length_);
			point_type bottom(-.01, -arm_length_);
			//the -.01 is for some edge case
			polygon_edges top_pivot_circle;
			quarter_circle_gen(arm_length_, 0, 0, 30, top_pivot_circle, true);
			polygon_edges bottom_pivot_circle;
			quarter_circle_gen(-arm_length_, 0, 0, 30, bottom_pivot_circle, false);
			top_pivot_circle.insert(top_pivot_circle.end(), bottom_pivot_circle.begin(), bottom_pivot_circle.end());
			top_pivot_circle.push_back(bottom);	
			top_pivot_circle.push_back(top);
			top_pivot_circle.push_back(top_pivot_circle[0]);
			
				
			boost::geometry::assign_points(pivot_circle, top_pivot_circle);
			//dis be contructor
			//TODO: make better
		}
		bool check_if_possible(point_type &cmd, bool &up_or_down, int check_type, double lift_height = 0)
		{		
			
			if(check_type < 2)
			{
				if(up_or_down)
				{
					if(boost::geometry::within(cmd, saved_polygons_[0]))
					{
						//ROS_WARN("success");
						return true;
					}
					else if(boost::geometry::within(cmd, saved_polygons_[1]))
					{
						//ROS_WARN("up to down");
						up_or_down = false;
						return false;
					}
					else
					{
						//ROS_WARN("FAIL");

						find_nearest_point(cmd, up_or_down, check_type, lift_height);
						return false;
					}
				}
				else
				{

					if(boost::geometry::within(cmd, saved_polygons_[1]))
					{
						//ROS_WARN("success");	
						return true;
					}
					else if(boost::geometry::within(cmd, saved_polygons_[0]))
					{
						//ROS_WARN("down to up");
						up_or_down = true;
						return false;
					}
					else
					{
						//ROS_WARN("FAIL");
						find_nearest_point(cmd, up_or_down, check_type, lift_height);
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
						//ROS_WARN("success");
						return true;
					}
					else
					{
						//ROS_WARN("FAIL");

						find_nearest_point(cmd, up_or_down, check_type);
						return false;
					}
				}
				else
				{

					if(boost::geometry::within(cmd, saved_polygons_[1]))
					{
						//ROS_WARN("success");	
						return true;
					}
					else
					{
						//ROS_WARN("FAIL");
						find_nearest_point(cmd, up_or_down, check_type);
						return false;
					}
				}
			}
		}
		bool safe_cmd(point_type &cmd, bool &up_or_down, bool &cmd_works, point_type &cur_pos, 
		bool &cur_up_or_down, double &hook_depth, double &hook_min_height, double &hook_max_height,
		point_type &cmd_return, bool &up_or_down_return)
		{
			
			//Note: uses heuristics only applicable to our robot

			//ROS_INFO_STREAM("cmd base check. Cmd: " << boost::geometry::wkt(cmd) << " up/down :" << up_or_down);
			cmd_works = check_if_possible(cmd, up_or_down, 0);
			
			cmd_return = cmd;
			up_or_down_return = up_or_down;

			check_if_possible(cur_pos, cur_up_or_down, 0);
			


			
			//ROS_INFO_STREAM("cur_pos check");

			//ROS_INFO_STREAM(" Cmd: " << boost::geometry::wkt(cur_pos) << " up/down :" << cur_up_or_down);
			//cur_pos.x(.05);
			//cur_pos.y(.5);
			//cur_up_or_down = false;	
			double cur_lift_height = cur_pos.y() - sin(acos(cur_pos.x()/arm_length_))*arm_length_
			*(cur_up_or_down ? 1 : -1); 
			
			double isolated_pivot_y =  sin(acos(cmd.x()/arm_length_))*arm_length_
			*( up_or_down ? 1 : -1) + cur_lift_height;
			//Could switch above to using circle func instead of trig func	
		
	
			double isolated_lift_delta_y = cmd.y() - isolated_pivot_y;

			double hook_current_height_delta = (cur_lift_height - min_extension_)/2;
			double hook_cmd_height_delta = (isolated_lift_delta_y)/2 + hook_current_height_delta;

			double y_low_hook_corner =  2 * (hook_min_height) - min_extension_ - sin(acos((hook_depth + 0.05)/arm_length_))*arm_length_;
			double y_high_hook_corner = 2 * (hook_max_height) - min_extension_ - sin(acos((hook_depth + 0.05)/arm_length_))*arm_length_;


			//hook_heights are for when arm is all the way down
			if((cmd.x() < hook_depth || cur_pos.x() < hook_depth) && 
			((cur_pos.y() > hook_current_height_delta + hook_min_height  
			&& cmd.y() < hook_max_height + hook_cmd_height_delta) || 
			(cmd.y() > hook_min_height + hook_cmd_height_delta 
			&& cur_pos.y() < hook_current_height_delta +  hook_max_height)))
			{
				//ROS_INFO_STREAM("cmd precheck. Cmd: " << boost::geometry::wkt(cmd) << " up/down :" << up_or_down);
				//ROS_WARN("HOOK LIMITED");
				if(cmd.x() < hook_depth)
				{
					cmd.x(hook_depth + .05); //adjust this arbitrary constant?
					if(cmd.y() > hook_min_height + hook_cmd_height_delta)
					{
						if(cur_pos.x() < hook_depth)
						{
							cmd.y(y_low_hook_corner);
						}
						else
						{
							cmd.y(cur_lift_height + isolated_lift_delta_y 
							+ sin(acos((hook_depth+.05)/arm_length_))*arm_length_
							*(up_or_down ? 1 : -1));
						}	
					}
					else
					{	
						if(cur_pos.x() < hook_depth)
						{
							cmd.y(y_high_hook_corner);
						}
						else
						{
							cmd.y(cur_lift_height + isolated_lift_delta_y 
							+ sin(acos((hook_depth+.05)/arm_length_))*arm_length_
							*(up_or_down ? 1 : -1));
						}	


					}
				}
				else
				{
					if(cmd.y() > hook_min_height + hook_cmd_height_delta)
					{
						cmd.y(y_low_hook_corner);
					}
					else
					{	
						cmd.y(y_high_hook_corner);
					}	
				}

				//Might need to take in account one more case, up to down



					
				//ROS_INFO_STREAM("cmd post check. Cmd: " << boost::geometry::wkt(cmd) << " up/down :" << up_or_down);
				
			}	
			
			isolated_pivot_y =  sin(acos(cmd.x()/arm_length_))*arm_length_
			*( up_or_down ? 1 : -1) + cur_lift_height;
			//Could switch above to using circle func instead of trig func	
			point_type test_pivot_cmd(cmd.x(), isolated_pivot_y);
		
	
			isolated_lift_delta_y = cmd.y() - isolated_pivot_y;
			
			//ROS_INFO_STREAM("pivot check. Cmd: " << boost::geometry::wkt(test_pivot_cmd) << " up/down :" << up_or_down << " lift_height: " << cur_lift_height);
			
			if(!check_if_possible(test_pivot_cmd, up_or_down, 1, cur_lift_height))
			{
				cmd.x(test_pivot_cmd.x());
				cmd.y(isolated_lift_delta_y + test_pivot_cmd.y());
				//ROS_INFO_STREAM("new pivot: " << boost::geometry::wkt(test_pivot_cmd) << " cmd: " << boost::geometry::wkt(cmd));
				return false;				
			}
			else
			{
				point_type test_lift_cmd(cur_pos.x(), cur_pos.y() + isolated_lift_delta_y);
				//ROS_INFO_STREAM("elevator check. Cmd: " << boost::geometry::wkt(test_lift_cmd) << " up/down :" << cur_up_or_down);
				if(!check_if_possible(test_lift_cmd, cur_up_or_down, 2))
				{
					cmd.y(test_lift_cmd.y() - cur_pos.y() + isolated_pivot_y);
					//ROS_INFO_STREAM("new elev: " << boost::geometry::wkt(test_lift_cmd) << " cmd: " << boost::geometry::wkt(cmd));
					return false;
				}
				else
				{
					//ROS_INFO_STREAM("cmd final check. Cmd: " << boost::geometry::wkt(cmd) << " up/down :" << up_or_down);
					return true;
				}
			} 
			
		}
	private:
		double min_extension_;
		double max_extension_;	
		double arm_length_;
		std::array<std::vector<linestring_type>, 2> poly_lines;
		polygon_type pivot_circle;
		std::array<polygon_type, 2> saved_polygons_;
		void find_nearest_point(point_type &cmd, bool &up_or_down, int check_type, double lift_height = 0)
		{
			//ROS_INFO_STREAM("finding nearest point for: " << boost::geometry::wkt(cmd) << " up/down: " << up_or_down << " check type: " << check_type);
			if(check_type == 0)
			{
				double min_dist_up  = 1000000;
				double min_dist_down  = 1000000;
				double temp_dist;
				int closest_point_up = 0;
				int closest_point_down = 0;
				bool closer_up_or_down = true;
				for(int i = 0; i < poly_lines[0].size(); i++)
				{
					temp_dist = boost::geometry::comparable_distance(cmd, poly_lines[0][i]);
					if(temp_dist < min_dist_up)
					{
						min_dist_up = temp_dist;
						closest_point_up = i;
					}
					//ROS_INFO_STREAM("current line: " << boost::geometry::wkt(poly_lines[0][i]) << " dist: " << temp_dist <<" up");

				}	
				for(int i = 0; i < poly_lines[1].size(); i++)
				{
					
					temp_dist = boost::geometry::comparable_distance(cmd, poly_lines[1][i]);
					if(temp_dist < min_dist_down)
					{
						min_dist_down = temp_dist;
						closest_point_down = i;
					}
					//ROS_INFO_STREAM("current line: " << boost::geometry::wkt(poly_lines[1][i]) << " dist: " << temp_dist <<" down");

				}	
				if(fabs(min_dist_down - min_dist_up) < .001)
				{
					closer_up_or_down = up_or_down;

				}
				else
				{
					closer_up_or_down = min_dist_up < min_dist_down;
					up_or_down = closer_up_or_down;
				}
				if(closer_up_or_down)
				{
					//ROS_INFO_STREAM("closest line is: " << boost::geometry::wkt(poly_lines[0][closest_point_up]) << " up");
					cmd =  find_closest_point_to_line(cmd, poly_lines[0][closest_point_up]);
				}	
				else
				{							
					//ROS_INFO_STREAM("closest line is: " << boost::geometry::wkt(poly_lines[1][closest_point_down]) << " down");
					cmd = find_closest_point_to_line(cmd, poly_lines[1][closest_point_down]);
				}	
			}
			else if(check_type == 1)
			{
				boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(0, lift_height);
				polygon_type new_pivot_circle;
				boost::geometry::transform(pivot_circle, new_pivot_circle, translate);
				vector<point_type> output_up;
				boost::geometry::intersection(new_pivot_circle, saved_polygons_[0], output_up);
				vector<point_type> output_down;
				boost::geometry::intersection(new_pivot_circle, saved_polygons_[1], output_down);

				//ROS_INFO_STREAM("circle: " << boost::geometry::wkt(new_pivot_circle));
				
			
				double min_dist_up  = 1000000;
				double min_dist_down  = 1000000;
				double temp_dist;
				int closest_point_up = 0;
				int closest_point_down = 0;
				bool closer_up_or_down = true;
				for(int i = 0; i < output_up.size(); i++)
				{
					
					temp_dist = boost::geometry::comparable_distance(cmd, output_up[i]);
					if(temp_dist < min_dist_up)
					{
						min_dist_up = temp_dist;
						closest_point_up = i;
					}
					//ROS_INFO_STREAM("circle point: " << boost::geometry::wkt(output_up[i]) << " up. dist : " << temp_dist);

				}	
				for(int i = 0; i < output_down.size(); i++)
				{
					temp_dist = boost::geometry::comparable_distance(cmd, output_down[i]);
					if(temp_dist < min_dist_down)
					{
						min_dist_down = temp_dist;
						closest_point_down = i;
					}
					//ROS_INFO_STREAM("circle point: " << boost::geometry::wkt(output_down[i]) << " down. dist : " << temp_dist);

				}
				if(fabs(min_dist_down - min_dist_up) < .001)
				{
					closer_up_or_down = up_or_down;

				}
				else
				{
					closer_up_or_down = min_dist_up < min_dist_down;
					up_or_down = closer_up_or_down;
				}
				if(closer_up_or_down)
				{
					if(output_up.size() != 0)
					{
					cmd = output_up[closest_point_up];
					}
					else
					{
					ROS_ERROR("INVALID PIVOT TEST");
					}
				}
				else
				{	
					if(output_down.size() != 0)
					{
					cmd = output_down[closest_point_down];
					}
					else
					{

					ROS_ERROR("INVALID PIVOT TEST");


					}
				}	

			}
			else
			{
				linestring_type up_line;
				point_type down_point(cmd.x(), -10);
				point_type up_point(cmd.x(), 100);
				up_line.push_back(down_point);
				up_line.push_back(up_point);
				
				vector<point_type> output;
			
				int id_up_down = up_or_down ? 0 : 1;
	
				boost::geometry::intersection(up_line, saved_polygons_[id_up_down], output);
				

					
				double min_dist  = 1000000;
				double temp_dist;
				int closest_point = 0;
				for(int i = 0; i < output.size(); i++)
				{
					temp_dist = boost::geometry::comparable_distance(cmd, output[i]);
					if(temp_dist < min_dist)
					{
						min_dist = temp_dist;
						closest_point = i;
					}

				}	
				if(output.size() != 0)
				{
				cmd =  output[closest_point];
				}
				else
				{
					ROS_ERROR("INVALID LIFT TEST");		
				}
			}	
			//Make this an actual function
			//If check_type is 1, we only rotate to get it in the zone
			//If check_type is 2, we only may alter y to get it in the zone
			//ROS_INFO_STREAM("Point is: " << boost::geometry::wkt(cmd) << " up/down: " << up_or_down << " check type: " << check_type);
			return;			
		}	
		point_type find_closest_point_to_line(point_type cmd, linestring_type line)
                {
                        double length_line = boost::geometry::length(line);

                        double project_dist = ((cmd.x() - line[1].x())*(line[0].x() - line[1].x()) + (cmd.y() - line[1].y())*(line[0].y() - line[1].y()))/length_line;
                        if(project_dist < 0)
                        {
                                return line[1];
                        }
                        else if(project_dist > length_line)
                        {
                                return line[0];
                        }
                        else
                        {
                                point_type point_on_line;
                                point_on_line.x(project_dist * (line[0].x() - line[1].x()) / length_line + line[1].x());
                                point_on_line.y(project_dist * (line[0].y() - line[1].y()) / length_line + line[1].y());
                                return point_on_line;
                        }
                }
		void quarter_circle_gen(double delta_height, double midpoint_z, double midpoint_x, int point_count,
		polygon_edges &circle, bool flip)
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
	
			//for(int i = 0; i < top_circle_up.size()	
			//insert into back line


			back_line_up.insert(back_line_up.end(), bottom_circle_up.begin(), bottom_circle_up.end());
			back_line_up.insert(back_line_up.end(), front_line_up.begin(), front_line_up.end());
			back_line_up.insert(back_line_up.end(), top_circle_up.begin(), top_circle_up.end());
			back_line_up.push_back(back_line_up[0]);


			back_line_down.insert(back_line_down.end(), bottom_circle_down.begin(), bottom_circle_down.end());
			back_line_down.insert(back_line_down.end(), front_line_down.begin(), front_line_down.end());
			back_line_down.insert(back_line_down.end(), top_circle_down.begin(), top_circle_down.end());
			back_line_down.push_back(back_line_down[0]);

			polygon_type down_poly;
			polygon_type up_poly;
			
			

			std::reverse(back_line_up.begin(), back_line_up.end());	
			std::reverse(back_line_down.begin(), back_line_down.end());	
			
			
			
			boost::geometry::assign_points(down_poly, back_line_down);
			boost::geometry::assign_points(up_poly, back_line_up);

			

			std::array<polygon_type, 2> up_and_down_polygons;
		
			std::list<polygon_type> output;			
			
			int i = 0;	
			
			output.clear();			
			boost::geometry::difference(down_poly, remove_zone_down, output);


			i = 0;
			BOOST_FOREACH(polygon_type const& p, output)
			{
				if(i == 0) {up_and_down_polygons[1] = p;
				
				}
				else{ROS_ERROR("Bad Remove Zone, REINSTALL WINDOWS?");}
				i++;
			}


			output.clear();			
			boost::geometry::difference(up_poly, remove_zone_down, output);
			
			i = 0;
			BOOST_FOREACH(polygon_type const& p, output)
			{
				if(i == 0) {up_and_down_polygons[0] = p;}
				else{ROS_ERROR("Bad Remove Zone, REINSTALL WINDOWS?");}
				i++;
			}
			output.clear();		
			
			polygon_edges up_edges;
			polygon_edges down_edges;
			for(auto it = boost::begin(boost::geometry::exterior_ring(up_and_down_polygons[0])); it != boost::end(boost::geometry::exterior_ring(up_and_down_polygons[0])); ++it)	
			{
				point_type temp_point;
				temp_point.x(boost::geometry::get<0>(*it));
				temp_point.y(boost::geometry::get<1>(*it));
				up_edges.push_back(temp_point);
			}
			for(auto it = boost::begin(boost::geometry::exterior_ring(up_and_down_polygons[1])); it != boost::end(boost::geometry::exterior_ring(up_and_down_polygons[1])); ++it)	
			{
				point_type temp_point;
				temp_point.x(boost::geometry::get<0>(*it));
				temp_point.y(boost::geometry::get<1>(*it));
				down_edges.push_back(temp_point);
			}
			for(int i = 0; i < up_edges.size() - 1; i++)
			{
				linestring_type temp_line;
				temp_line.push_back(up_edges[i]);
				temp_line.push_back(up_edges[i+1]);
				poly_lines[0].push_back(temp_line);
			} 
			for(int i = 0; i < down_edges.size() - 1; i++)
			{
				linestring_type temp_line;
				temp_line.push_back(down_edges[i]);
				temp_line.push_back(down_edges[i+1]);
				poly_lines[1].push_back(temp_line);
			} 
	
			ROS_INFO_STREAM("remove_zone: " << boost::geometry::wkt(remove_zone_down));
			ROS_INFO_STREAM("poly up: " << boost::geometry::wkt(up_and_down_polygons[0]));
			ROS_INFO_STREAM("poly down: " << boost::geometry::wkt(up_and_down_polygons[1]));
			//TODO: remove excluded points from poly
			//Also, test these polys with print statements
			

			return up_and_down_polygons;
		}
};
}

