/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the FRCRobot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#pragma once

#include <thread>
#include <ros_control_boilerplate/frc_robot_interface.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <IterativeRobotBase.h>
#include <DriverStation.h>
#include <realtime_tools/realtime_publisher.h>
#include <NidecBrushless.h>
#include <DigitalInput.h>
#include <DigitalOutput.h>
#include <SafePWM.h>
#include <Solenoid.h>
#include <DoubleSolenoid.h>
#include <AHRS.h>
#include <Compressor.h>
#include <PowerDistributionPanel.h>
#include "LiveWindow/LiveWindow.h"
#include "SmartDashboard/SmartDashboard.h"

namespace frcrobot_control
{
	class ROSRobot;
/// \brief Hardware interface for a robot
class FRCRobotHWInterface : public ros_control_boilerplate::FRCRobotInterface
{
	public:
		/**
		 * \brief Constructor
		 * \param nh - Node handle for topics.
		 */
		FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		~FRCRobotHWInterface();

		/** \brief Initialize the hardware interface */
		virtual void init(void) override;

		/** \brief Read the state from the robot hardware. */
		virtual void read(ros::Duration &elapsed_time) override;

		/** \brief Write the command to the robot hardware. */
		virtual void write(ros::Duration &elapsed_time) override;

	protected:
		void hal_keepalive_thread(void);
		void process_motion_profile_buffer_thread(ros::Rate rate);

	private:
		/** Get conversion factor for position, velocity, and closed-loop stuff */
		virtual double getConversionFactor(int encoder_cycle_per_revolution, hardware_interface::FeedbackDevice encoder_feedback, hardware_interface::TalonMode talon_mode, int joint_id);

		bool convertControlMode(const hardware_interface::TalonMode input_mode,
								ctre::phoenix::motorcontrol::ControlMode &output_mode);
		bool convertNeutralMode(const hardware_interface::NeutralMode input_mode,
								ctre::phoenix::motorcontrol::NeutralMode &output_mode);
		bool convertFeedbackDevice(
			const hardware_interface::FeedbackDevice input_fd,
			ctre::phoenix::motorcontrol::FeedbackDevice &output_fd);
		bool convertLimitSwitchSource(
			const hardware_interface::LimitSwitchSource input_ls,
			ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls);
		bool convertLimitSwitchNormal(
			const hardware_interface::LimitSwitchNormal input_ls,
			ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls);
		bool safeTalonCall(ctre::phoenix::ErrorCode error_code, 
				const std::string &talon_method_name);

		std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX>> can_talons_;
		std::vector<std::shared_ptr<frc::NidecBrushless>> nidec_brushlesses_;
		std::vector<std::shared_ptr<frc::DigitalInput>> digital_inputs_;
		std::vector<std::shared_ptr<frc::DigitalOutput>> digital_outputs_;
		std::vector<std::shared_ptr<frc::SafePWM>> PWMs_;
		std::vector<std::shared_ptr<frc::Solenoid>> solenoids_;
		std::vector<std::shared_ptr<frc::DoubleSolenoid>> double_solenoids_;
		std::vector<std::shared_ptr<AHRS>> navXs_;
		std::vector<std::shared_ptr<frc::AnalogInput>> analog_inputs_;
		std::vector<std::shared_ptr<frc::Compressor>> compressors_;

		std::thread motion_profile_thread_;
		bool        run_motion_profile_thread_;

		//PowerDistributionPanel pdp_joint_;
		std::shared_ptr<ROSRobot> robot_;
};  // class

}  // namespace

