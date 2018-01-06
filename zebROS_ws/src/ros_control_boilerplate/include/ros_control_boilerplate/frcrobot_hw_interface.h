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
#include <IterativeRobot.h>
#include <DriverStation.h>
#include <realtime_tools/realtime_publisher.h>
#include <NidecBrushless.h>
#include <DigitalInput.h>
#include <DigitalOutput.h>
#include <SafePWM.h>

namespace frcrobot_control
{
class ROSIterativeRobot : public frc::IterativeRobot
{
	public:
		void StartCompetition(void)
		{
			RobotInit();
			HAL_ObserveUserProgramStarting();
		}
		void OneIteration(void)
		{
			// wait for driver station data so the loop doesn't hog the CPU
			DriverStation::GetInstance().WaitForData();
			LoopFunc();
		}
};

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
		virtual void init(void);

		/** \brief Read the state from the robot hardware. */
		virtual void read(ros::Duration &elapsed_time);

		virtual double getRadiansConversionFactor(hardware_interface::FeedbackDevice encoder_feedback, int joint_id);
		virtual double getRadiansPerSecConversionFactor(hardware_interface::FeedbackDevice encoder_feedback, int joint_id);

		/** \brief Write the command to the robot hardware. */
		virtual void write(ros::Duration &elapsed_time);

		/** \brief Enforce limits for all values before writing */
		virtual void enforceLimits(ros::Duration &period);

	protected:
		void hal_keepalive_thread(void);

	private:
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

		std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX>> can_talons_;
		std::vector<std::shared_ptr<frc::NidecBrushless>> nidec_brushlesses_;
		std::vector<std::shared_ptr<frc::DigitalInput>> digital_inputs_;
		std::vector<std::shared_ptr<frc::DigitalOutput>> digital_outputs_;
		std::vector<std::shared_ptr<frc::SafePWM>> PWMs_;

		std::thread hal_thread_;
		bool        run_hal_thread_;

		ROSIterativeRobot robot_;

};  // class

}  // namespace

