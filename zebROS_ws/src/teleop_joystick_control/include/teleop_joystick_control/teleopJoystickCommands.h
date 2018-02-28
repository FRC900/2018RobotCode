#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include "ros_control_boilerplate/JoystickState.h"
#include "teleop_joystick_control/RobotState.h"
#include "talon_controllers/CloseLoopControllerMsg.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "ros/time.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <string>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void evaluateCommands(const ros_control_boilerplate::JoystickState::ConstPtr &msg);
void rumbleTypeConverterPublish(uint16_t leftRumble, uint16_t rightRumble);
void evaluateTime(const ros_control_boilerplate::MatchSpecificData::ConstPtr &msg);
void navXCallback(const sensor_msgs::Imu &navXState);
void cubeCallback(const std_msgs::Bool &cube);

extern double navX_angle_;
extern int navX_index_;
extern ros::Subscriber navX_heading_;
extern ros::Subscriber cube_state_;
