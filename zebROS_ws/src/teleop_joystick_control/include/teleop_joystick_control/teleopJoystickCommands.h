#pragma once

#include "ros/ros.h"
#include "ros_control_boilerplate/JoystickState.h"
#include "talon_controllers/CloseLoopControllerMsg.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble);
void navXCallback(const sensor_msgs::Imu &navXState);
void cubeCallback(const std_msgs::Bool &cube);
void overrideCallback(const std_msgs::Bool &override_lim);

