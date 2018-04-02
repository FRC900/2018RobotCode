#pragma once

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <robot_visualizer/RobotVisualizeState.h>

void rotate_polygon(geometry_msgs::Polygon &poly, double angle);
void translate_polygon(geometry_msgs::Polygon &poly, double x, double y);
void plot_robot_cb(const robot_visualizer::RobotVisualizeState &robot_viz);

