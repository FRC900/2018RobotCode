/*
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar
 */

#include <cmath>

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <urdf/urdfdom_compatibility.h>

#include <boost/assign.hpp>

#include <talon_swerve_drive_controller/swerve_drive_controller.h>

#include <Eigen/Dense>

//TODO: include swerve stuff from C-Control
using Eigen::Vector2d;
using std::array;
using Eigen::Affine2d;
using Eigen::Matrix2d;
using Eigen::Vector2d;

using ros::Time;
using geometry_msgs::TwistConstPtr;
using ros::Duration;

const std::string talon_swerve_drive_controller::TalonSwerveDriveController::DEF_BASE_LINK = "base_link";
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_ODOM_PUB_FREQ = 50.;
const bool talon_swerve_drive_controller::TalonSwerveDriveController::DEF_PUB_ODOM_TO_BASE = false;
const std::string talon_swerve_drive_controller::TalonSwerveDriveController::DEF_ODOM_FRAME = "odom";
const std::string talon_swerve_drive_controller::TalonSwerveDriveController::DEF_BASE_FRAME = "base_link";
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_INIT_X = 0.;
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_INIT_Y = 0.;
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_INIT_YAW = 0.;
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_SD = 0.01;

/*
static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}
*/
/*
* \brief Check that a link exists and has a geometry collision.
* \param link The link
* \return true if the link has a collision element with geometry
*/
static bool hasCollisionGeometry(const urdf::LinkConstSharedPtr &link)
{
	if (!link)
	{
		ROS_ERROR("Link == NULL.");
		return false;
	}

	if (!link->collision)
	{
		ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
		return false;
	}

	if (!link->collision->geometry)
	{
		ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
		return false;
	}
	return true;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr &link)
{
	if (!hasCollisionGeometry(link))
	{
		return false;
	}

	if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
	{
		ROS_DEBUG_STREAM("Link " << link->name << " does not have cylinder geometry");
		return false;
	}

	return true;
}

/*
 * \brief Check if the link is modeled as a sphere
	return true;
}

/*
 * \brief Check if the link is modeled as a sphere
	return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 *
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */
static bool isSphere(const urdf::LinkConstSharedPtr &link)
{
	if (!hasCollisionGeometry(link))
	{
		return false;
	}

	if (link->collision->geometry->type != urdf::Geometry::SPHERE)
	{
		ROS_DEBUG_STREAM("Link " << link->name << " does not have sphere geometry");
		return false;
	}

	return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr &wheel_link, double &wheel_radius)
{
	if (isCylinder(wheel_link))
	{
		wheel_radius = (static_cast<urdf::Cylinder *>(wheel_link->collision->geometry.get()))->radius;
		return true;
	}
	else if (isSphere(wheel_link))
	{
		wheel_radius = (static_cast<urdf::Sphere *>(wheel_link->collision->geometry.get()))->radius;
		return true;
	}

	ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder or sphere!");
	return false;
}

namespace talon_swerve_drive_controller
{


TalonSwerveDriveController::TalonSwerveDriveController() :
	open_loop_(false),
	wheel_radius_(0.0),
	cmd_vel_timeout_(0.5), //Change to 5.0 for auto path planning testing
	allow_multiple_cmd_vel_publishers_(true),
	base_frame_id_("base_link"),
	odom_frame_id_("odom"),
	enable_odom_tf_(true),
	wheel_joints_size_(0),
	publish_cmd_(false)

	//model_({0, 0, 0, 0, 0, 0}),
	//invertWheelAngle_(false),
	//units_({1,1,1,1,1,1}),
	//driveRatios_({0, 0, 0}),
	//units_({0, 0, 0, 0})
{
}

bool TalonSwerveDriveController::init(hardware_interface::TalonCommandInterface *hw,
									  ros::NodeHandle &root_nh,
									  ros::NodeHandle &controller_nh)
{
	const std::string complete_ns = controller_nh.getNamespace();
	std::size_t id = complete_ns.find_last_of("/");
	name_ = complete_ns.substr(id + 1);

	// Get joint names from the parameter server
	std::vector<std::string> speed_names, steering_names;
	if (!getWheelNames(controller_nh, "speed", speed_names) or
			!getWheelNames(controller_nh, "steering", steering_names))
	{
		return false;
	}

	if (speed_names.size() != steering_names.size())
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "#speed (" << speed_names.size() << ") != " <<
							   "#steering (" << steering_names.size() << ").");
		return false;
	}
	else
	{
		wheel_joints_size_ = speed_names.size();

		speed_joints_.resize(wheel_joints_size_);
		steering_joints_.resize(wheel_joints_size_);
	}

	// Odometry related:
	double publish_rate;
	std::string base_link;
        controller_nh.param("base_link", base_link, DEF_BASE_LINK);
	controller_nh.param("publish_rate", publish_rate, 50.0);
	ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
						  << publish_rate << "Hz.");
	publish_period_ = ros::Duration(1.0 / publish_rate);






	// Publish limited velocity:
	//controller_nh.param("publish_cmd", publish_cmd_, publish_cmd_);

	// TODO : see if model_, driveRatios, units can be local instead of member vars
	// If either parameter is not available, we need to look up the value in the URDF
	//bool lookup_wheel_coordinates = !controller_nh.getParam("wheel_coordinates", wheel_coordinates_);
	bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
	bool lookup_max_speed = !controller_nh.getParam("max_speed", model_.maxSpeed);
	bool lookup_mass = !controller_nh.getParam("mass", model_.mass);
	bool lookup_motor_free_speed = !controller_nh.getParam("motor_free_speed", model_.motorFreeSpeed);
	bool lookup_motor_stall_torque = !controller_nh.getParam("motor_stall_torque", model_.motorStallTorque);
	// TODO : why not just use the number of wheels read from yaml?
	bool lookup_motor_quantity = !controller_nh.getParam("motor_quantity", model_.motorQuantity);
	bool lookup_invert_wheel_angle = !controller_nh.getParam("invert_wheel_angle", invertWheelAngle_);
	bool lookup_ratio_encoder_to_rotations = !controller_nh.getParam("ratio_encoder_to_rotations", driveRatios_.encodertoRotations);
	bool lookup_ratio_motor_to_rotations = !controller_nh.getParam("ratio_motor_to_rotations", driveRatios_.motortoRotations);
	bool lookup_ratio_motor_to_steering = !controller_nh.getParam("ratio_motor_to_steering", driveRatios_.motortoSteering); // TODO : not used?
	bool lookup_encoder_drive_get_V_units = !controller_nh.getParam("encoder_drive_get_V_units", units_.rotationGetV);
	bool lookup_encoder_drive_get_P_units = !controller_nh.getParam("encoder_drive_get_P_units", units_.rotationGetP);
	bool lookup_encoder_drive_set_V_units = !controller_nh.getParam("encoder_drive_set_V_units", units_.rotationSetV);
	bool lookup_encoder_drive_set_P_units = !controller_nh.getParam("encoder_drive_set_P_units", units_.rotationSetP);
	bool lookup_encoder_steering_get_units = !controller_nh.getParam("encoder_steering_get_units", units_.steeringGet);
	bool lookup_encoder_steering_set_units = !controller_nh.getParam("encoder_steering_set_units", units_.steeringSet);
	std::vector<double> wheel1a;
	std::vector<double> wheel2a;
	std::vector<double> wheel3a;
	std::vector<double> wheel4a;
	bool lookup_wheel1x = !controller_nh.getParam("wheel_coords1x", wheel_coords_[0][0]);
	bool lookup_wheel2x = !controller_nh.getParam("wheel_coords2x", wheel_coords_[1][0]);
	bool lookup_wheel3x = !controller_nh.getParam("wheel_coords3x", wheel_coords_[2][0]);
	bool lookup_wheel4x = !controller_nh.getParam("wheel_coords4x", wheel_coords_[3][0]);
	bool lookup_wheel1y = !controller_nh.getParam("wheel_coords1y", wheel_coords_[0][1]);
	bool lookup_wheel2y = !controller_nh.getParam("wheel_coords2y", wheel_coords_[1][1]);
	bool lookup_wheel3y = !controller_nh.getParam("wheel_coords3y", wheel_coords_[2][1]);
	bool lookup_wheel4y = !controller_nh.getParam("wheel_coords4y", wheel_coords_[3][1]);





	ROS_INFO_STREAM("Coords: " << wheel_coords_[0] << "   "<< wheel_coords_[1] << "   "<< wheel_coords_[2] << "   "<< wheel_coords_[3]);
	std::vector<double> offsets;
	for (auto it = steering_names.cbegin(); it != steering_names.cend(); ++it)
	{
		ros::NodeHandle nh(controller_nh, *it);
		double dbl_val = 0;
		if (!nh.getParam("offset", dbl_val))
			ROS_ERROR_STREAM("Can not read offset for " << *it);
		offsets.push_back(dbl_val);
	}

	/*
	if (!setOdomParamsFromUrdf(root_nh,
	                          speed_names[0],
	                          steering_names[0],
	                          //lookup_wheel_coordinates,
	                          lookup_wheel_radius))
	{
	  return false;
	}

	// Regardless of how we got the separation and radius, use them
	*/
	// to set the odometry parameters
	//setOdomPubFields(root_nh, controller_nh);

	/*if (publish_cmd_)
	{
	  cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 100));
	}
	*/
	// Get the joint object to use in the realtime loop

	// TODO : all of these need to be read from params
	/*
	model.maxSpeed = 3.3528;
	model.mass = 70;
	model.motorFreeSpeed = 5330;
	model.motorStallTorque = 2.41;
	model.motorQuantity = 4;
	*/
	model_.wheelRadius =  wheel_radius_;



	/*
	invertWheelAngle(false);
	swerveVar::ratios driveRatios({20, 7, 7});
	swerveVar::encoderUnits units({1,1,1,1,1,1});
	*/

	swerveC_ = std::make_shared<swerve>(wheel_coords_, offsets, invertWheelAngle_, driveRatios_, units_, model_);
	for (int i = 0; i < wheel_joints_size_; ++i)
	{
		ROS_INFO_STREAM_NAMED(name_,
							  "Adding speed motors with joint name: " << speed_names[i]
							  << " and steering motors with joint name: " << steering_names[i]);

		ros::NodeHandle l_nh(controller_nh, speed_names[i]);
		speed_joints_[i].initWithNode(hw, nullptr, l_nh);
		ros::NodeHandle r_nh(controller_nh, steering_names[i]);
		steering_joints_[i].initWithNode(hw, nullptr, r_nh);
	}



	sub_command_ = controller_nh.subscribe("cmd_vel", 1, &TalonSwerveDriveController::cmdVelCallback, this);
	motion_profile_serv_ = controller_nh.advertiseService("run_profile", &TalonSwerveDriveController::motionProfileService, this);
	//sub_run_profile_ = controller_nh.subscribe("run_profile", 1, &TalonSwerveDriveController::runCallback, this);


	double odom_pub_freq;
        controller_nh.param("odometry_publishing_frequency", odom_pub_freq, DEF_ODOM_PUB_FREQ);

	comp_odom_ = odom_pub_freq > 0;
	//ROS_WARN("COMPUTING ODOM");
	if (comp_odom_)
	{
		odom_pub_period_ = Duration(1 / odom_pub_freq);
		controller_nh.param("publish_odometry_to_base_transform", pub_odom_to_base_,
				DEF_PUB_ODOM_TO_BASE);

		double init_x, init_y, init_yaw;
		controller_nh.param("initial_x", init_x, DEF_INIT_X);
		controller_nh.param("initial_y", init_y, DEF_INIT_Y);
		controller_nh.param("initial_yaw", init_yaw, DEF_INIT_YAW);
		double x_sd, y_sd, yaw_sd;
		controller_nh.param("x_sd", x_sd, DEF_SD);
		controller_nh.param("y_sd", y_sd, DEF_SD);
		controller_nh.param("yaw_sd", yaw_sd, DEF_SD);
		double x_speed_sd, y_speed_sd, yaw_speed_sd;
		controller_nh.param("x_speed_sd", x_speed_sd, DEF_SD);
		controller_nh.param("y_speed_sd", y_speed_sd, DEF_SD);
		controller_nh.param("yaw_speed_sd", yaw_speed_sd, DEF_SD);

		init_odom_to_base_.setIdentity();
		init_odom_to_base_.rotate(init_yaw);
		init_odom_to_base_.translation() = Vector2d(init_x, init_y);
		odom_to_base_ = init_odom_to_base_;
		odom_rigid_transf_.setIdentity();

		wheel_pos_.resize(2, WHEELCOUNT);
		//ROS_WARN("working h");
		for(size_t i = 0; i < WHEELCOUNT; i++)
		{
			//ROS_INFO_STREAM("id: " << i << "pos" << wheel_coords_[i]);
			wheel_pos_.col(i) = wheel_coords_[i];
			//ROS_WARN("f1.test");
		}


		const Vector2d centroid = wheel_pos_.rowwise().mean();
		wheel_pos_.colwise() -= centroid;
		neg_wheel_centroid_ = -centroid;

		new_wheel_pos_.resize(WHEELCOUNT, 2);

		std::string odom_frame, base_frame;
		controller_nh.param("odometry_frame", odom_frame, DEF_ODOM_FRAME);
		controller_nh.param("base_frame", base_frame, DEF_BASE_FRAME);

		odom_pub_.msg_.header.frame_id = odom_frame;
		odom_pub_.msg_.child_frame_id = base_frame;

		odom_pub_.msg_.pose.pose.position.z = 0;

		odom_pub_.msg_.pose.covariance.assign(0);
		odom_pub_.msg_.pose.covariance[0] = x_sd * x_sd;
		odom_pub_.msg_.pose.covariance[7] = y_sd * y_sd;
		odom_pub_.msg_.pose.covariance[35] = yaw_sd * yaw_sd;

		odom_pub_.msg_.twist.twist.linear.z = 0;
		odom_pub_.msg_.twist.twist.angular.x = 0;
		odom_pub_.msg_.twist.twist.angular.y = 0;

		odom_pub_.msg_.twist.covariance.assign(0);
		odom_pub_.msg_.twist.covariance[0] = x_speed_sd * x_speed_sd;
		odom_pub_.msg_.twist.covariance[7] = y_speed_sd * y_speed_sd;
		odom_pub_.msg_.twist.covariance[35] = yaw_speed_sd * yaw_speed_sd;
		odom_pub_.init(controller_nh, "odom", 1);

		if (pub_odom_to_base_)
		{
			odom_tf_pub_.msg_.transforms.resize(1);
			geometry_msgs::TransformStamped& odom_tf_trans =
				odom_tf_pub_.msg_.transforms[0];
			odom_tf_trans.header.frame_id = odom_pub_.msg_.header.frame_id;
			odom_tf_trans.child_frame_id = odom_pub_.msg_.child_frame_id;
			odom_tf_trans.transform.translation.z = 0;
			odom_tf_pub_.init(controller_nh, "/tf", 1);
		}

		for (size_t row = 0; row < WHEELCOUNT; row++)
		{
			old_wheel_pos_[row] = {0, 0};
			last_wheel_rot_[row] = speed_joints_[row].getPosition();
		}
	}

	return true;
}

void TalonSwerveDriveController::compOdometry(const Time& time, const double inv_delta_t)
{
	//ROS_INFO_STREAM("WORKS");
	// Compute the rigid transform from wheel_pos_ to new_wheel_pos_.

	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		const double new_wheel_rot = speed_joints_[k].getPosition();
		const double delta_rot = new_wheel_rot - last_wheel_rot_[k];
		//int inverterD = (k%2==0) ? -1 : 1;
		const double dist = -delta_rot * wheel_radius_ * driveRatios_.encodertoRotations; //* inverterD;
		//NOTE: below is a hack, TODO: REMOVE

		const double steer_angle = swerveC_->getWheelAngle(k, steering_joints_[k].getPosition());
		const Eigen::Vector2d delta_pos = {-dist*sin(steer_angle), dist*cos(steer_angle)};
		new_wheel_pos_(k, 0) = wheel_coords_[k][0] + delta_pos[0];
		new_wheel_pos_(k, 1) = wheel_coords_[k][1] + delta_pos[1];

		//ROS_INFO_STREAM("id: " << k << " delta: " << delta_pos << " steer: " << steer_angle << " dist: " << dist);
		last_wheel_rot_[k] = new_wheel_rot;
	}

	const Eigen::RowVector2d new_wheel_centroid =
		new_wheel_pos_.colwise().mean();
	new_wheel_pos_.rowwise() -= new_wheel_centroid;

	//ROS_INFO_STREAM("rows: " << wheel_pos_.rows() << " cols: " << wheel_pos_.cols());
	//ROS_INFO_STREAM("neg wheel centroid" << neg_wheel_centroid_ << " new centroid: " << new_wheel_centroid);

	const Matrix2d h = wheel_pos_ * new_wheel_pos_;
	const Eigen::JacobiSVD<Matrix2d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Matrix2d rot = svd.matrixV() * svd.matrixU().transpose();
	if (rot.determinant() < 0)
		rot.col(1) *= -1;

	odom_rigid_transf_.matrix().block(0, 0, 2, 2) = rot;
	odom_rigid_transf_.translation() =
		rot * neg_wheel_centroid_ + new_wheel_centroid.transpose();
	odom_to_base_ = odom_to_base_ * odom_rigid_transf_;

	const double odom_x = odom_to_base_.translation().x();
	const double odom_y = odom_to_base_.translation().y();
	const double odom_yaw = atan2(odom_to_base_(1, 0), odom_to_base_(0, 0));

	//ROS_INFO_STREAM("odom_x: " << odom_x << " odom_y: " << odom_y << " odom_yaw: " << odom_yaw);
	// Publish the odometry.
	//TODO CHECK THIS PUB

	geometry_msgs::Quaternion orientation;
	bool orientation_comped = false;

	// tf
	if (pub_odom_to_base_ && time - last_odom_tf_pub_time_ >= odom_pub_period_ &&
			odom_tf_pub_.trylock())
	{
		orientation = tf::createQuaternionMsgFromYaw(odom_yaw);
		orientation_comped = true;

		geometry_msgs::TransformStamped& odom_tf_trans =
			odom_tf_pub_.msg_.transforms[0];
		odom_tf_trans.header.stamp = time;
		odom_tf_trans.transform.translation.x = odom_x;
		odom_tf_trans.transform.translation.y = odom_y;
		odom_tf_trans.transform.rotation = orientation;
		ROS_INFO_STREAM(odom_x);
		odom_tf_pub_.unlockAndPublish();
		last_odom_tf_pub_time_ = time;
	}
	// odom
	if (time - last_odom_pub_time_ >= odom_pub_period_ && odom_pub_.trylock())
	{
		if (!orientation_comped)
			orientation = tf::createQuaternionMsgFromYaw(odom_yaw);

		odom_pub_.msg_.header.stamp = time;
		odom_pub_.msg_.pose.pose.position.x = odom_x;
		odom_pub_.msg_.pose.pose.position.y = odom_y;
		odom_pub_.msg_.pose.pose.orientation = orientation;

		odom_pub_.msg_.twist.twist.linear.x =
			odom_rigid_transf_.translation().x() * inv_delta_t;
		odom_pub_.msg_.twist.twist.linear.y =
			odom_rigid_transf_.translation().y() * inv_delta_t;
		odom_pub_.msg_.twist.twist.angular.z =
			atan2(odom_rigid_transf_(1, 0), odom_rigid_transf_(0, 0)) * inv_delta_t;

		odom_pub_.unlockAndPublish();
		last_odom_pub_time_ = time;
	}
}


void TalonSwerveDriveController::update(const ros::Time &time, const ros::Duration &period)
{
	const double delta_t = period.toSec();
	const double inv_delta_t = 1 / delta_t;
	if (comp_odom_) compOdometry(time, inv_delta_t);

	/*
	// COMPUTE AND PUBLISH ODOMETRY
	if (open_loop_)
	{
	  odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
	}
	else
	{
	  double left_pos  = 0.0;
	  double right_pos = 0.0;
	  for (size_t i = 0; i < wheel_joints_size_; ++i)
	  {
	    const double lp = speed_joints_[i].getPosition();
	    const double rp = steering_joints_[i].getPosition();
	    if (std::isnan(lp) || std::isnan(rp))
	      return;

	    left_pos  += lp;
	    right_pos += rp;
	  }
	  left_pos  /= wheel_joints_size_;
	  right_pos /= wheel_joints_size_;

	  // Estimate linear and angular velocity using joint information
	  odometry_.update(left_pos, right_pos, time);
	}

	// Publish odometry message
	if (last_state_publish_time_ + publish_period_ < time)
	{
	  last_state_publish_time_ += publish_period_;
	  // Compute and store orientation info
	  const geometry_msgs::Quaternion orientation(
	        tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

	  // Populate odom message and publish
	  if (odom_pub_->trylock())
	  {
	    odom_pub_->msg_.header.stamp = time;
	    odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
	    odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
	    odom_pub_->msg_.pose.pose.orientation = orientation;
	    odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
	    odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
	    odom_pub_->unlockAndPublish();
	  }

	  // Publish tf /odom frame
	  if (enable_odom_tf_ && tf_odom_pub_->trylock())
	  {
	    geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
	    odom_frame.header.stamp = time;
	    odom_frame.transform.translation.x = odometry_.getX();
	    odom_frame.transform.translation.y = odometry_.getY();
	    odom_frame.transform.rotation = orientation;
	    tf_odom_pub_->unlockAndPublish();
	  }
	}
	*/
	// MOVE ROBOT
	// Retreive current velocity command and time step:

	//ROS_INFO_STREAM("mode: " << *(mode_.readFromRT())); 

	if(*(mode_.readFromRT()))
	{
		Commands curr_cmd = *(command_.readFromRT());
		const double dt = (time - curr_cmd.stamp).toSec();

		for (size_t i = 0; i < wheel_joints_size_; ++i)
		{
			speed_joints_[i].setMode(velocity_mode);
			steering_joints_[i].setMode(position_mode);
		}
		// Brake if cmd_vel has timeout:
		if (dt > cmd_vel_timeout_)
		{
			curr_cmd.lin = {0.0, 0.0};
			curr_cmd.ang = 0.0;
		}

		if (fabs(curr_cmd.lin[0]) <= 1e-6 && fabs(curr_cmd.lin[1]) <= 1e-6 && fabs(curr_cmd.ang) <= 1e-6)
		{
			brake();
			return;
		}

		// Limit velocities and accelerations:
		const double cmd_dt(period.toSec());

		// Compute wheels velocities:
		//Parse curr_cmd to get velocity vector and rotation (z axis)
		//TODO: check unit conversions/coordinate frames

		array<double, WHEELCOUNT> curPos;
		for (int i = 0; i < WHEELCOUNT; i++)
			curPos[i] = steering_joints_[i].getPosition();
		std::array<bool, WHEELCOUNT> holder;
		std::array<Vector2d, WHEELCOUNT> speeds_angles  = swerveC_->motorOutputs(curr_cmd.lin, curr_cmd.ang, M_PI/2, false, holder, false, curPos, true);
		
		// Set wheels velocities:
		for (size_t i = 0; i < wheel_joints_size_; ++i)
		{
			//ROS_INFO_STREAM("id:" << i << " speed: " <<speeds_angles[i][0]);
			speed_joints_[i].setCommand(speeds_angles[i][0]);
			steering_joints_[i].setCommand(speeds_angles[i][1]);
		}
	}
	else
	{	
		for (size_t i = 0; i < wheel_joints_size_; ++i)
		{
			speed_joints_[i].setMode(motion_profile_mode);
			steering_joints_[i].setMode(motion_profile_mode);
		}

		const int set_on  = *(run_.readFromRT()) ? 1 : 0;
		for(size_t i = 0; i < WHEELCOUNT; i++)
		{
			speed_joints_[i].setCommand(set_on);
			steering_joints_[i].setCommand(set_on);
		}
	}
	
	if(*(buffer_.readFromRT()))
	{
		buffer_ = false;
		
		cmd_points curr_cmd = *(command_points_.readFromRT());

		ROS_WARN("BUFFERING");
		//TODO: optimize code?
		array<double, WHEELCOUNT> curPos;
		for (int i = 0; i < WHEELCOUNT; i++)
			curPos[i] = steering_joints_[i].getPosition();

		std::array<bool, WHEELCOUNT> holder;
		
		std::array<Vector2d, WHEELCOUNT> angles_positions  = swerveC_->motorOutputs(curr_cmd.lin_points_pos[0], curr_cmd.ang_pos[0], M_PI/2 + curr_cmd.ang_pos[0], false, holder, false, curPos, false);
			//TODO: angles on the velocity array below are superfluous, could remove
		std::array<Vector2d, WHEELCOUNT> angles_velocities  = swerveC_->motorOutputs(curr_cmd.lin_points_vel[0], curr_cmd.ang_vel[0], M_PI/2 + curr_cmd.ang_pos[0], false, holder, false, curPos, false);
		for (size_t i = 0; i < WHEELCOUNT; i++)
			curPos[i] = angles_positions[i][1];
		//Do first point and initialize stuff

		for(size_t i = 0; i < WHEELCOUNT; i++)
		{


			//steering_joints_[i].clearMotionProfileTrajectories();
			//speed_joints_[i].clearMotionProfileTrajectories();

			steering_joints_[i].clearMotionProfileHasUnderrun();
			speed_joints_[i].clearMotionProfileHasUnderrun();

			holder_points_[i][0].position = angles_positions[i][0];
			holder_points_[i][1].position = angles_positions[i][1];
			holder_points_[i][0].velocity = angles_velocities[i][0];
			holder_points_[i][1].velocity = 0; //TODO: FIX
			holder_points_[i][0].isLastPoint = false;
			holder_points_[i][1].isLastPoint = false;
			holder_points_[i][0].zeroPos = true;
			holder_points_[i][1].zeroPos = false;
			holder_points_[i][0].trajectoryDuration = curr_cmd.dt;
			holder_points_[i][1].trajectoryDuration = curr_cmd.dt;

			speed_joints_[i].pushMotionProfileTrajectory(holder_points_[i][0]);
			steering_joints_[i].pushMotionProfileTrajectory(holder_points_[i][1]);
			holder_points_[i][0].zeroPos = false;
		}



		const int point_count = curr_cmd.lin_points_pos.size();
		for(size_t i = 0; i < point_count - 2; i++)
		{
			angles_positions  = swerveC_->motorOutputs(curr_cmd.lin_points_pos[i+1] - curr_cmd.lin_points_pos[i], curr_cmd.ang_pos[i+1] - curr_cmd.ang_pos[i], M_PI/2 + curr_cmd.ang_pos[i+1], false, holder, false, curPos, false);
			//TODO: angles on the velocity array below are superfluous, could remove
			angles_velocities  = swerveC_->motorOutputs(curr_cmd.lin_points_vel[i], curr_cmd.ang_vel[i], M_PI/2 + curr_cmd.ang_pos[i+1], false, holder, false, curPos, false);
			for (size_t k = 0; k < WHEELCOUNT; k++)
				curPos[k] = angles_positions[k][1];

			for(size_t k = 0; k < WHEELCOUNT; k++)
			{
				holder_points_[k][0].position += angles_positions[k][0];
				holder_points_[k][1].position = angles_positions[k][1];
				holder_points_[k][0].velocity = angles_velocities[k][0];
				
				ROS_INFO_STREAM("speed. Pos: " << holder_points_[k][0].position << " vel: " << holder_points_[k][0].velocity);
				ROS_INFO_STREAM("steering. Pos: " << holder_points_[k][1].position << " vel: " << holder_points_[k][1].velocity);

				speed_joints_[k].pushMotionProfileTrajectory(holder_points_[k][0]);
				steering_joints_[k].pushMotionProfileTrajectory(holder_points_[k][1]);
			}
		}
		//Final Setter
		angles_positions  = swerveC_->motorOutputs(curr_cmd.lin_points_pos[point_count-1] - curr_cmd.lin_points_pos[point_count-2], curr_cmd.ang_pos[point_count-1] - curr_cmd.ang_pos[point_count-2], M_PI/2+ curr_cmd.ang_pos[point_count-1], false, holder, false, curPos, false);
		//TODO: angles on the velocity array below are superfluous, could remove
		angles_velocities  = swerveC_->motorOutputs(curr_cmd.lin_points_vel[point_count - 1], curr_cmd.ang_vel[point_count - 1], M_PI/2 + curr_cmd.ang_pos[point_count-1], false, holder, false, curPos, false);
		for(size_t k = 0; k < WHEELCOUNT; k++)
		{
			holder_points_[k][0].position += angles_positions[k][0];
			holder_points_[k][1].position = angles_positions[k][1];
			holder_points_[k][0].velocity = angles_velocities[k][0];

			holder_points_[k][0].isLastPoint = true;
			holder_points_[k][1].isLastPoint = true;

			speed_joints_[k].pushMotionProfileTrajectory(holder_points_[k][0]);
			steering_joints_[k].pushMotionProfileTrajectory(holder_points_[k][1]);
		}
	}
}

void TalonSwerveDriveController::starting(const ros::Time &time)
{
	brake();

	// Register starting time used to keep fixed rate
	if (comp_odom_)
	{
		last_odom_pub_time_ = time;
		last_odom_tf_pub_time_ = time;
	}
	//odometry_.init(time);
}

void TalonSwerveDriveController::stopping(const ros::Time & /*time*/)
{
	brake();
}

void TalonSwerveDriveController::brake()
{
	//required input, but not needed in this case
	array<bool, WHEELCOUNT> hold;
	//Use parking config

	array<double, WHEELCOUNT> curPos;
	for (int i = 0; i < WHEELCOUNT; i++)
	{
		curPos[i] = steering_joints_[i].getPosition();
	}
	std::array<Vector2d, WHEELCOUNT> park = swerveC_->motorOutputs({0, 0}, 0, 0, false, hold, true, curPos, false);
	for (size_t i = 0; i < wheel_joints_size_; ++i)
	{
		speed_joints_[i].setCommand(0.0);
		steering_joints_[i].setCommand(park[i][1]);
	}
}



void TalonSwerveDriveController::cmdVelCallback(const geometry_msgs::Twist &command)
{
	if (isRunning())
	{
		// check that we don't have multiple publishers on the command topic
		if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
		{
			ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
											<< " publishers. Only 1 publisher is allowed. Going to brake.");
			brake();
			return;
		}
		
		if(command.linear.z != 0)
		{
			ROS_WARN("Rotors not up to speed!");
		}
		if(command.angular.x != 0 | command.angular.y != 0)
		{
			ROS_WARN("Reaction wheels need alignment. Please reverse polarity on neutron flux capacitor");
		}
		if(command.linear.x > 3.0*pow(10, 8) | command.linear.y > 3.0*pow(10, 8) | command.linear.z > 3.0*pow(10, 8))
		{
			ROS_WARN("PHYSICS VIOLATION DETECTED. DISABLE TELEPORTATION UNIT!");
		}

		//TODO change to twist msg
		
		command_struct_.ang = command.angular.z;
		command_struct_.lin[0] = command.linear.x;
		command_struct_.lin[1] = command.linear.y;
		command_struct_.stamp = ros::Time::now();
		command_.writeFromNonRT (command_struct_);
		
		mode_.writeFromNonRT (true);

		
		//TODO fix debug
		ROS_DEBUG_STREAM_NAMED(name_,
							  "Added values to command. "
							  << "Ang: "   << command_struct_.ang << ", "
							  << "Lin X: "   << command_struct_.lin[0] << ", "
							  << "Lin Y: "   << command_struct_.lin[1] << ", "
							  << "Stamp: " << command_struct_.stamp);
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}

bool TalonSwerveDriveController::motionProfileService(talon_swerve_drive_controller::MotionProfile::Request &req, talon_swerve_drive_controller::MotionProfile::Response &res)
{
	if (isRunning())
	{
		/*
		// check that we don't have multiple publishers on the command topic
		if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
		{
			ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
											<< " publishers. Only 1 publisher is allowed. Going to brake.");
			brake();
			return;
		}
		*/		

		if(req.buffer)
		{
			points_struct_.lin_points_pos.clear();
			points_struct_.lin_points_vel.clear();
			points_struct_.ang_pos.clear();
			points_struct_.ang_vel.clear();
			double duration = req.joint_trajectory.points[1].time_from_start.toSec()
			- req.joint_trajectory.points[0].time_from_start.toSec();

			if(duration < .0025)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_0ms;
			}
			else if(duration < .0075)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_5ms;
			}
			else if(duration < .015)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_10ms;
			}
			else if(duration < .025)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_20ms;
			}
			else if(duration < .035)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_30ms;
			}
			else if(duration < .045)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_40ms;
			}
			else if(duration < .075)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_50ms;
			}
			else
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_100ms;
			}
			for(size_t i = 0; i < req.joint_trajectory.points.size(); i++)
			{
				points_struct_.lin_points_pos.push_back({req.joint_trajectory.points[i].positions[0], req.joint_trajectory.points[i].positions[1]});
				points_struct_.lin_points_vel.push_back({req.joint_trajectory.points[i].velocities[0], req.joint_trajectory.points[i].velocities[1]});
				points_struct_.ang_pos.push_back(req.joint_trajectory.points[i].positions[2]);
				points_struct_.ang_vel.push_back(req.joint_trajectory.points[i].velocities[2]);
			}
			command_points_.writeFromNonRT(points_struct_);

		}
		buffer_.writeFromNonRT(req.buffer);
		mode_.writeFromNonRT(!(req.mode || req.run));
		run_.writeFromNonRT(req.run);
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}


/*
void TalonSwerveDriveController::cmdCallback(const talon_swerve_drive_controller::CompleteCmd &command)
{
	if (isRunning())
	{
		// check that we don't have multiple publishers on the command topic
		if (sub_command_.getNumPublishers() > 1)
		{
			ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
											<< " publishers. Only 1 publisher is allowed. Going to brake.");
			brake();
			return;
		}
		mode_.writeFromNonRT(command.cmd_vel_or_points);
		if(command.cmd_vel_or_points)
		{
			command_struct_.ang = command.twist_.angular.z;
			command_struct_.lin[0] = command.twist_.linear.x;
			command_struct_.lin[1] = command.twist_.linear.y;
			command_struct_.stamp = ros::Time::now();
			command_.writeFromNonRT (command_struct_);
		}
		else
		{
			points_struct_.lin_points_pos.clear();
			points_struct_.lin_points_vel.clear();
			points_struct_.ang_pos.clear();
			points_struct_.ang_vel.clear();
			double duration = command.joint_trajectory.points[1].time_from_start.toSec()
			- command.joint_trajectory.points[0].time_from_start.toSec();

			if(duration < .0025)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_0ms;
			}
			else if(duration < .0075)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_5ms;
			}
			else if(duration < .015)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_10ms;
			}
			else if(duration < .025)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_20ms;
			}
			else if(duration < .035)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_30ms;
			}
			else if(duration < .045)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_40ms;
			}
			else if(duration < .075)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_50ms;
			}
			else
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_100ms;
			}
			for(size_t i = 0; i < command.joint_trajectory.points.size(); i++)
			{
				points_struct_.lin_points_pos.push_back({command.joint_trajectory.points[i].positions[0], command.joint_trajectory.points[i].positions[1]});
				points_struct_.lin_points_vel.push_back({command.joint_trajectory.points[i].velocities[0], command.joint_trajectory.points[i].velocities[1]});
				points_struct_.ang_pos.push_back(command.joint_trajectory.points[i].positions[2]);
				points_struct_.ang_vel.push_back(command.joint_trajectory.points[i].velocities[2]);
			}
			command_points_.writeFromNonRT(points_struct_);
		}
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}

*/

bool TalonSwerveDriveController::getWheelNames(ros::NodeHandle &controller_nh,
		const std::string &wheel_param,
		std::vector<std::string> &wheel_names)
{
	XmlRpc::XmlRpcValue wheel_list;
	if (!controller_nh.getParam(wheel_param, wheel_list))
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "Couldn't retrieve wheel param '" << wheel_param << "'.");
		return false;
	}

	if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		if (wheel_list.size() == 0)
		{
			ROS_ERROR_STREAM_NAMED(name_,
								   "Wheel param '" << wheel_param << "' is an empty list");
			return false;
		}

		for (int i = 0; i < wheel_list.size(); ++i)
		{
			if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
			{
				ROS_ERROR_STREAM_NAMED(name_,
									   "Wheel param '" << wheel_param << "' #" << i <<
									   " isn't a string.");
				return false;
			}
		}

		wheel_names.resize(wheel_list.size());
		for (int i = 0; i < wheel_list.size(); ++i)
		{
			wheel_names[i] = static_cast<std::string>(wheel_list[i]);
		}
	}
	else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
	{
		wheel_names.push_back(wheel_list);
	}
	else
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "Wheel param '" << wheel_param <<
							   "' is neither a list of strings nor a string.");
		return false;
	}

	return true;
}
}
/*
  bool TalonSwerveDriveController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                             const std::string& steering_name,
                             const std::string& speed_name,
                             bool lookup_wheel_radius)
  //{
    if (!(lookup_wheel_radius))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR_NAMED(name_, "Robot descripion couldn't be retrieved from param server.");
      return false;
    }

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

    //TODO: replace with swerve equivalent
    //urdf::JointConstSharedPtr left_wheel_joint(model->getJoint(left_wheel_name));
    //urdf::JointConstSharedPtr right_wheel_joint(model->getJoint(right_wheel_name));






    if(lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model->getLink(left_wheel_joint->child_link_name), wheel_radius_))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << left_wheel_name << " wheel radius");
        return false;
      }
    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  }
*/
//}

