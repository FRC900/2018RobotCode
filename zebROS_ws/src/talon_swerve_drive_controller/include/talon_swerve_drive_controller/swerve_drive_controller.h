/*********************************************************************
 * Software License Agreement (BSD License)
 *
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
 * Author: Enrique Fernández
 */

#pragma once


#include <talon_swerve_drive_controller/MotionProfile.h>
#include <std_msgs/Bool.h>
#include <string>
#include <controller_interface/controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

//#include <talon_swerve_drive_controller/odometry.h>
#include <talon_swerve_drive_controller/speed_limiter.h>

#include <functional>
#include <talon_swerve_drive_controller/Swerve.h>
#include <talon_swerve_drive_controller/Blank.h>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include <talon_swerve_drive_controller/MotionProfilePoints.h>

namespace talon_swerve_drive_controller
{
const hardware_interface::TalonMode motion_profile_mode = hardware_interface::TalonMode::TalonMode_MotionProfile;
const hardware_interface::TalonMode velocity_mode = hardware_interface::TalonMode::TalonMode_Velocity;
//const hardware_interface::TalonMode neutral_mode = hardware_interface::TalonMode::TalonMode_Neutral;
const hardware_interface::TalonMode position_mode = hardware_interface::TalonMode::TalonMode_Position;

/**
 * This class makes some assumptions on the model of the robot:
 *  - the rotation axes of wheels are collinear
 *  - the wheels are identical in radius
 * Additional assumptions to not duplicate information readily available in the URDF:
 *  - the wheels have the same parent frame
 *  - a wheel collision geometry is a cylinder or sphere in the urdf
 *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
*/
class TalonSwerveDriveController
	: public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:
		TalonSwerveDriveController();

		/**
		 * \brief Initialize controller
		 * \param hw            Velocity joint interface for the wheels
		 * \param root_nh       Node handle at root namespace
		 * \param controller_nh Node handle inside the controller namespace
		 */
		bool init(hardware_interface::TalonCommandInterface *hw,
				  ros::NodeHandle &root_nh,
				  ros::NodeHandle &controller_nh);

		/**
		 * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
		 * \param time   Current time
		 * \param period Time since the last called to update
		 */
		void update(const ros::Time &time, const ros::Duration &period);

		/**
		 * \brief Starts controller
		 * \param time Current time
		 */
		void starting(const ros::Time &time);

		/**
		 * \brief Stops controller
		 * \param time Current time
		 */
		void stopping(const ros::Time & /*time*/);

	private:
		Eigen::Vector2d wheel1;
        	Eigen::Vector2d wheel2;
        	Eigen::Vector2d wheel3;
        	Eigen::Vector2d wheel4;

		int set_check_;

		void compOdometry(const ros::Time& time, const double inv_delta_t);
		Eigen::MatrixX2d new_wheel_pos_;	
		std::array<Eigen::Vector2d, WHEELCOUNT> old_wheel_pos_; //	
		std::array<double, WHEELCOUNT> last_wheel_rot_;	//

		Eigen::Vector2d neg_wheel_centroid_;
		bool comp_odom_;

		std::string name_;

		/// Odometry related:
		ros::Duration publish_period_;
		ros::Time last_state_publish_time_;
		bool open_loop_;

		std::shared_ptr<swerve> swerveC_;

		/// Hardware handles:
		//TODO: IMPORTANT, make generalized, and check
		std::vector<talon_controllers::TalonControllerInterface> speed_joints_;
		std::vector<talon_controllers::TalonControllerInterface> steering_joints_;
		/// Velocity command related:

		struct Commands
		{
			Eigen::Vector2d lin;
			double ang;
			ros::Time stamp;

			Commands() : lin({0.0, 0.0}), ang(0.0), stamp(0.0) {}
		};
		struct cmd_points
		{	
			std::vector<std::vector<double>> drive_pos;
			std::vector<std::vector<double>> drive_vel;
			std::vector<std::vector<double>> steer_pos;
			hardware_interface::TrajectoryDuration dt;
			int half_dt;
		};
		
		realtime_tools::RealtimeBuffer<bool> mode_;
		realtime_tools::RealtimeBuffer<bool> buffer_;
		realtime_tools::RealtimeBuffer<bool> clear_;
		realtime_tools::RealtimeBuffer<Commands> command_;
		Commands command_struct_;
		Commands brake_struct_;
		realtime_tools::RealtimeBuffer<cmd_points> command_points_;
		cmd_points points_struct_;
		
		ros::Subscriber sub_command_;



		ros::ServiceServer motion_profile_serv_;
		ros::ServiceServer brake_serv_;
	
		
		std::array<std::array<hardware_interface::TrajectoryPoint, 2>, WHEELCOUNT> holder_points_;
	
		realtime_tools::RealtimeBuffer<bool> run_;

		hardware_interface::TalonMode motion_profile = hardware_interface::TalonMode::TalonMode_MotionProfile;
		hardware_interface::TalonMode velocity_mode = hardware_interface::TalonMode::TalonMode_Velocity;
        	hardware_interface::TalonMode position_mode = hardware_interface::TalonMode::TalonMode_MotionMagic;

		/// Publish executed commands
		//boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

		//Odometry related:
		//boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
		//boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
		//Odometry odometry_;

		/// Wheel radius (assuming it's the same for the left and right wheels):
		double wheel_radius_;
		
		swerveVar::driveModel model_;
		
		bool invertWheelAngle_;

		swerveVar::ratios driveRatios_;
		
		swerveVar::encoderUnits units_;
		double f_static_;

		/// Timeout to consider cmd_vel commands old:
		double cmd_vel_timeout_;

		/// Whether to allow multiple publishers on cmd_vel topic or not:
		bool allow_multiple_cmd_vel_publishers_;

		/// Frame to use for the robot base:
		std::string base_frame_id_;

		/// Frame to use for odometry and odom tf:
		std::string odom_frame_id_;

		/// Whether to publish odometry to tf or not:
		bool enable_odom_tf_;

		/// Number of wheel joints:
		size_t wheel_joints_size_;

		/// Speed limiters:
		Commands last1_cmd_;
		Commands last0_cmd_;

		/// Publish limited velocity:
		bool publish_cmd_;

		/**
		 * \brief Brakes the wheels, i.e. sets the velocity to 0
		 * RG: also sets to parking config
		 */
		void brake();

		/**
		 * \brief Velocity command callback
		 * \param command Velocity command message (twist)
		 */
		void cmdVelCallback(const geometry_msgs::Twist &command);
		bool motionProfileService(talon_swerve_drive_controller::MotionProfilePoints::Request &req, talon_swerve_drive_controller::MotionProfilePoints::Response &res);
		bool brakeService(talon_swerve_drive_controller::Blank::Request &req, talon_swerve_drive_controller::Blank::Response &res);

		/**
		 * \brief Get the wheel names from a wheel param
		 * \param [in]  controller_nh Controller node handler
		 * \param [in]  wheel_param   Param name
		 * \param [out] wheel_names   Vector with the whel names
		 * \return true if the wheel_param is available and the wheel_names are
		 *        retrieved successfully from the param server; false otherwise
		 */
		bool getWheelNames(ros::NodeHandle &controller_nh,
						   const std::string &wheel_param,
						   std::vector<std::string> &wheel_names);

		/**
		 * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
		 * \param root_nh Root node handle
		 * \param left_wheel_name Name of the left wheel joint
		 * \param right_wheel_name Name of the right wheel joint
		 */
		/*
		
		bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
		                            const std::string& steering_name,
		                            const std::string& speed_name,
		                            bool lookup_wheel_radius);

		 */
		/**
		* \brief Sets the odometry publishing fields
		* \param root_nh Root node handle
		* \param controller_nh Node handle inside the controller namespace
		*/
		/*
		void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
		*/

		static const std::string DEF_ROBOT_DESC_NAME;
		static const std::string DEF_BASE_LINK;
		static const double DEF_CMD_VEL_TIMEOUT;

		static const double DEF_LIN_SPEED_LIMIT;
		static const double DEF_LIN_ACCEL_LIMIT;
		static const double DEF_LIN_DECEL_LIMIT;

		static const double DEF_YAW_SPEED_LIMIT;
		static const double DEF_YAW_ACCEL_LIMIT;
		static const double DEF_YAW_DECEL_LIMIT;

		static const double DEF_FULL_AXLE_SPEED_ANG;
		static const double DEF_ZERO_AXLE_SPEED_ANG;

		static const double DEF_WHEEL_DIA_SCALE;

		static const double DEF_ODOM_PUB_FREQ;
		static const bool DEF_PUB_ODOM_TO_BASE;
		static const std::string DEF_ODOM_FRAME;
		static const std::string DEF_BASE_FRAME;
		static const double DEF_INIT_X;
		static const double DEF_INIT_Y;
		static const double DEF_INIT_YAW;
		static const double DEF_SD;

		std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords_;
		
//		static const Eigen::Vector2d X_DIR;
		
		bool pub_odom_to_base_;       // Publish the odometry to base frame transform
		ros::Duration odom_pub_period_;    // Odometry publishing period
		Eigen::Affine2d init_odom_to_base_;  // Initial odometry to base frame transform
		Eigen::Affine2d odom_to_base_;       // Odometry to base frame transform
		Eigen::Affine2d odom_rigid_transf_;
		Eigen::Matrix2Xd wheel_pos_;

		realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
		realtime_tools::RealtimePublisher<tf::tfMessage> odom_tf_pub_;
		ros::Time last_odom_pub_time_, last_odom_tf_pub_time_;
};

PLUGINLIB_EXPORT_CLASS(talon_swerve_drive_controller::TalonSwerveDriveController, controller_interface::ControllerBase);

} // namespace diff_drive_controller
