/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Original Author: Dave Coleman
   Desc:   Helper ros_control hardware interface that loads configurations
*/

#include <ros_control_boilerplate/frc_robot_interface.h>
#include <limits>

namespace ros_control_boilerplate
{
FRCRobotInterface::FRCRobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) :
   	  name_("generic_hw_interface")
	, nh_(nh)
	, num_can_talon_srxs_(0)
	, num_nidec_brushlesses_(0)
	, num_digital_inputs_(0)
	, num_digital_outputs_(0)
	, num_pwm_(0)
	, num_solenoids_(0)
	, num_double_solenoids_(0)
	, num_rumble_(0)
	, match_time_state_(0)
{
	// Check if the URDF model needs to be loaded
	if (urdf_model == NULL)
		loadURDF(nh, "robot_description");
	else
		urdf_model_ = urdf_model;

	// Load rosparams
	ros::NodeHandle rpnh(nh_, "hardware_interface"); // TODO(davetcoleman): change the namespace to "frc_robot_interface" aka name_

	// Read a list of joint information from ROS parameters.  Each entry in the list
	// specifies a name for the joint and a hardware ID corresponding
	// to that value.  Joint types and locations are specified (by name)
	// in a URDF file loaded along with the controller.
	XmlRpc::XmlRpcValue joint_param_list;
	if (!rpnh.getParam("joints", joint_param_list))
		throw std::runtime_error("No joints were specified.");
	for (int i = 0; i < joint_param_list.size(); i++)
	{
		XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
		if (!joint_params.hasMember("name"))
			throw std::runtime_error("A joint name was not specified");
		XmlRpc::XmlRpcValue &xml_joint_name = joint_params["name"];
		if (!xml_joint_name.valid() ||
				xml_joint_name.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint name was specified (expecting a string)");
		const std::string joint_name = xml_joint_name;

		if (!joint_params.hasMember("type"))
			throw std::runtime_error("A joint type was not specified");
		XmlRpc::XmlRpcValue &xml_joint_type = joint_params["type"];
		if (!xml_joint_type.valid() ||
				xml_joint_type.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint type was specified (expecting a string).");
		const std::string joint_type = xml_joint_type;

		if (joint_type == "can_talon_srx")
		{
			if (!joint_params.hasMember("can_id"))
				throw std::runtime_error("A CAN Talon SRX can_id was not specified");
			XmlRpc::XmlRpcValue &xml_can_id = joint_params["can_id"];
			if (!xml_can_id.valid() ||
					xml_can_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint can_id was specified (expecting an int).");
			const int can_id = xml_can_id;

			can_talon_srx_names_.push_back(joint_name);
			can_talon_srx_can_ids_.push_back(can_id);
		}
		else if (joint_type == "nidec_brushless")
		{
			if (!joint_params.hasMember("pwm_channel"))
				throw std::runtime_error("A Nidec Brushless pwm_channel was not specified");
			XmlRpc::XmlRpcValue &xml_pwm_channel = joint_params["pwm_channel"];
			if (!xml_pwm_channel.valid() ||
					xml_pwm_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint pwm_channel was specified (expecting an int).");
			const int pwm_channel = xml_pwm_channel;

			if (!joint_params.hasMember("dio_channel"))
				throw std::runtime_error("A Nidec Brushless dio_channel was not specified");
			XmlRpc::XmlRpcValue &xml_dio_channel = joint_params["dio_channel"];
			if (!xml_dio_channel.valid() ||
					xml_dio_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint dio_channel was specified (expecting an int).");
			const int dio_channel = xml_dio_channel;

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
						xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid joint invert was specified (expecting a boolean).");
				invert = xml_invert;
			}

			nidec_brushless_names_.push_back(joint_name);
			nidec_brushless_pwm_channels_.push_back(pwm_channel);
			nidec_brushless_dio_channels_.push_back(dio_channel);
			nidec_brushless_inverts_.push_back(invert);
		}
		else if (joint_type == "digital_input")
		{
			if (!joint_params.hasMember("dio_channel"))
				throw std::runtime_error("A Digital Input dio_channel was not specified");
			XmlRpc::XmlRpcValue &xml_digital_input_dio_channel = joint_params["dio_channel"];
			if (!xml_digital_input_dio_channel.valid() ||
					xml_digital_input_dio_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint dio_channel was specified (expecting an int).");

			const int digital_input_dio_channel = xml_digital_input_dio_channel;

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
						xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid joint invert was specified (expecting a boolean).");
				invert = xml_invert;
			}

			digital_input_names_.push_back(joint_name);
			digital_input_dio_channels_.push_back(digital_input_dio_channel);
			digital_input_inverts_.push_back(invert);
		}
		else if (joint_type == "digital_output")
		{
			if (!joint_params.hasMember("dio_channel"))
				throw std::runtime_error("A Digital Output dio_channel was not specified");
			XmlRpc::XmlRpcValue &xml_digital_output_dio_channel = joint_params["dio_channel"];
			if (!xml_digital_output_dio_channel.valid() ||
					xml_digital_output_dio_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint dio_channel was specified (expecting an int).");

			const int digital_output_dio_channel = xml_digital_output_dio_channel;

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
						xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid joint invert was specified (expecting a boolean).");
				invert = xml_invert;
			}

			digital_output_names_.push_back(joint_name);
			digital_output_dio_channels_.push_back(digital_output_dio_channel);
			digital_output_inverts_.push_back(invert);
		}
		else if (joint_type == "pwm")
		{
			if (!joint_params.hasMember("pwm_channel"))
				throw std::runtime_error("A PWM pwm_channel was not specified");
			XmlRpc::XmlRpcValue &xml_pwm_pwm_channel = joint_params["pwm_channel"];
			if (!xml_pwm_pwm_channel.valid() ||
					xml_pwm_pwm_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint pwm_channel was specified (expecting an int).");

			const int pwm_pwm_channel = xml_pwm_pwm_channel;

			bool invert = false;
			if (joint_params.hasMember("invert"))
			{
				XmlRpc::XmlRpcValue &xml_invert = joint_params["invert"];
				if (!xml_invert.valid() ||
						xml_invert.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
					throw std::runtime_error("An invalid joint invert was specified (expecting a boolean).");
				invert = xml_invert;
			}

			pwm_names_.push_back(joint_name);
			pwm_pwm_channels_.push_back(pwm_pwm_channel);
			pwm_inverts_.push_back(invert);
		}
		else if (joint_type == "solenoid")
		{
			if (!joint_params.hasMember("id"))
				throw std::runtime_error("A solenoid id was not specified");
			XmlRpc::XmlRpcValue &xml_solenoid_id = joint_params["id"];
			if (!xml_solenoid_id.valid() ||
					xml_solenoid_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint solenoid id was specified (expecting an int).");

			const int solenoid_id = xml_solenoid_id;


			solenoid_names_.push_back(joint_name);
			solenoid_ids_.push_back(solenoid_id);
		}
		else if (joint_type == "double_solenoid")
		{
			if (!joint_params.hasMember("forward_id"))
				throw std::runtime_error("A double_solenoid forward_id was not specified");
			XmlRpc::XmlRpcValue &xml_double_solenoid_forward_id = joint_params["forward_id"];
			if (!xml_double_solenoid_forward_id.valid() ||
					xml_double_solenoid_forward_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint double solenoid forward_id was specified (expecting an int).");

			const int double_solenoid_forward_id = xml_double_solenoid_forward_id;

			if (!joint_params.hasMember("reverse_id"))
				throw std::runtime_error("A double_solenoid reverse_id was not specified");
			XmlRpc::XmlRpcValue &xml_double_solenoid_reverse_id = joint_params["reverse_id"];
			if (!xml_double_solenoid_reverse_id.valid() ||
					xml_double_solenoid_reverse_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint double solenoid reverse_id was specified (expecting an int).");

			const int double_solenoid_reverse_id = xml_double_solenoid_reverse_id;

			double_solenoid_names_.push_back(joint_name);
			double_solenoid_forward_ids_.push_back(double_solenoid_forward_id);
			double_solenoid_reverse_ids_.push_back(double_solenoid_reverse_id);
		}
		else if (joint_type == "rumble")
		{
			if (!joint_params.hasMember("rumble_port"))
				throw std::runtime_error("A rumble_port was not specified");
			XmlRpc::XmlRpcValue &xml_rumble_port = joint_params["rumble_port"];
			if (!xml_rumble_port.valid() ||
					xml_rumble_port.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint rumble_port was specified (expecting an int).");

			const int rumble_port = xml_rumble_port;

			rumble_names_.push_back(joint_name);
			rumble_ports_.push_back(rumble_port);
		}
		else if (joint_type == "navX")
		{
			if (!joint_params.hasMember("id"))
				throw std::runtime_error("A navX id was not specified");
			XmlRpc::XmlRpcValue &xml_navX_id = joint_params["id"];
			if (!xml_navX_id.valid() ||
					xml_navX_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint id was specified (expecting an int).");

			const int navX_id = xml_navX_id;

			navX_names_.push_back(joint_name);
			navX_ids_.push_back(navX_id);
		}
		else
		{
			std::stringstream s;
			s << "Unknown joint type " << joint_type << " specified";
			throw std::runtime_error(s.str());
		}
	}
}

void FRCRobotInterface::init()
{
	num_can_talon_srxs_ = can_talon_srx_names_.size();
	// Create vectors of the correct size for
	// talon HW state and commands
	talon_command_.resize(num_can_talon_srxs_);

	// Loop through the list of joint names
	// specified as params for the hardware_interface.
	// For each of them, create a Talon object. This
	// object is used to send and recieve commands
	// and status to/from the physical Talon motor
	// controller on the robot.  Use this pointer
	// to initialize each Talon with various params
	// set for that motor controller in config files.
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering Talon Interface for " << can_talon_srx_names_[i] << " at hw ID " << can_talon_srx_can_ids_[i]);

		// Create joint state interface
		// Also register as JointStateInterface so that legacy
		// ROS code which uses that object type can
		// access basic state info from the talon
		// Code which needs more specific status should
		// get a TalonStateHandle instead.
		talon_state_.push_back(hardware_interface::TalonHWState(can_talon_srx_can_ids_[i]));
	}
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		// Create state interface for the given Talon
		// and point it to the data stored in the
		// corresponding talon_state array entry
		hardware_interface::TalonStateHandle tsh(can_talon_srx_names_[i], &talon_state_[i]);
		talon_state_interface_.registerHandle(tsh);

		// Do the same for a command interface for
		// the same talon
		hardware_interface::TalonCommandHandle tch(tsh, &talon_command_[i]);
		talon_command_interface_.registerHandle(tch);
	}

	// Set vectors to correct size to hold data
	// for each of the brushless motors we're trying
	// to control
	num_nidec_brushlesses_ = nidec_brushless_names_.size();
	brushless_pos_.resize(num_nidec_brushlesses_);
	brushless_vel_.resize(num_nidec_brushlesses_);
	brushless_eff_.resize(num_nidec_brushlesses_);
	brushless_command_.resize(num_nidec_brushlesses_);
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << nidec_brushless_names_[i] << " at PWM channel " << nidec_brushless_pwm_channels_[i] << " / DIO channel " << nidec_brushless_dio_channels_[i]);
		// Create state interface for the given brushless motor
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::JointStateHandle jsh(nidec_brushless_names_[i], &brushless_pos_[i], &brushless_vel_[i], &brushless_eff_[i]);
		joint_state_interface_.registerHandle(jsh);

		// Do the same for a command interface for
		// the same brushless motor
		hardware_interface::JointHandle jh(jsh, &brushless_command_[i]);
		joint_velocity_interface_.registerHandle(jh);
	}

	num_digital_inputs_ = digital_input_names_.size();
	digital_input_state_.resize(num_digital_inputs_);
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << digital_input_names_[i] << " at DIO channel " << digital_input_dio_channels_[i] << " / invert " << digital_input_inverts_[i]);
		// Create state interface for the given digital input
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::JointStateHandle dish(digital_input_names_[i], &digital_input_state_[i], &digital_input_state_[i], &digital_input_state_[i]);
		joint_state_interface_.registerHandle(dish);

	}

	num_digital_outputs_ = digital_output_names_.size();
	digital_output_state_.resize(num_digital_outputs_);
	digital_output_command_.resize(num_digital_outputs_);
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << digital_output_names_[i] << " at DIO channel " << digital_output_dio_channels_[i] << " / invert " << digital_output_inverts_[i]);

		hardware_interface::JointStateHandle dosh(digital_output_names_[i], &digital_output_state_[i], &digital_output_state_[i], &digital_output_state_[i]);
		joint_state_interface_.registerHandle(dosh);

		// Do the same for a command interface for
		// the digital output
		hardware_interface::JointHandle doh(dosh, &digital_output_command_[i]);
		joint_velocity_interface_.registerHandle(doh);
	}

	num_pwm_ = pwm_names_.size();
	pwm_state_.resize(num_pwm_);
	pwm_command_.resize(num_pwm_);
	for (size_t i = 0; i < num_pwm_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << pwm_names_[i] << " at PWM channel " << pwm_pwm_channels_[i] << " / invert " << pwm_inverts_[i]);

		hardware_interface::JointStateHandle psh(pwm_names_[i], &pwm_state_[i], &pwm_state_[i], &pwm_state_[i]);
		joint_state_interface_.registerHandle(psh);

		// Do the same for a command interface for
		// the same brushless motor
		hardware_interface::JointHandle ph(psh, &pwm_command_[i]);
		joint_velocity_interface_.registerHandle(ph);
	}
	num_solenoids_ = solenoid_names_.size();
        solenoid_state_.resize(num_solenoids_);
        solenoid_command_.resize(num_solenoids_);
        for (size_t i = 0; i < num_solenoids_; i++)
        {
                ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << solenoid_names_[i] << " at id " << solenoid_ids_[i]);

                hardware_interface::JointStateHandle ssh(solenoid_names_[i], &solenoid_state_[i], &solenoid_state_[i], &solenoid_state_[i]);
                joint_state_interface_.registerHandle(ssh);

                // Do the same for a command interface for
                // the digital output
                hardware_interface::JointHandle soh(ssh, &solenoid_command_[i]);
                joint_velocity_interface_.registerHandle(soh);
        }

	num_double_solenoids_ = double_solenoid_names_.size();
        double_solenoid_state_.resize(num_double_solenoids_);
        double_solenoid_command_.resize(num_double_solenoids_);
        for (size_t i = 0; i < num_double_solenoids_; i++)
        {
                ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << double_solenoid_names_[i] << " at forward id" << double_solenoid_forward_ids_[i] << "at reverse id" << double_solenoid_reverse_ids_[i]);

                hardware_interface::JointStateHandle dssh(double_solenoid_names_[i], &double_solenoid_state_[i], &double_solenoid_state_[i], &double_solenoid_state_[i]);
                joint_state_interface_.registerHandle(dssh);

                // Do the same for a command interface for
                // the digital output
                hardware_interface::JointHandle dsoh(dssh, &double_solenoid_command_[i]);
                joint_velocity_interface_.registerHandle(dsoh);
        }
	num_rumble_ = rumble_names_.size();
	rumble_state_.resize(num_rumble_);
	rumble_command_.resize(num_rumble_);
	for (size_t i = 0; i < num_rumble_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << rumble_names_[i] << " at port " <<rumble_ports_[i]);

		hardware_interface::JointStateHandle rsh(rumble_names_[i], &rumble_state_[i], &rumble_state_[i], &rumble_state_[i]);
		joint_state_interface_.registerHandle(rsh);

		// Do the same for a command interface for
		// the same brushless motor
		hardware_interface::JointHandle rh(rsh, &rumble_command_[i]);
		joint_velocity_interface_.registerHandle(rh);
	}
	num_navX_ = navX_names_.size();
	navX_state_.resize(num_navX_);
	for (size_t i = 0; i < num_navX_; i++)
	{
		navX_state_.push_back(hardware_interface::ImuSensorHandle());
	}
	for (size_t i = 0; i < num_navX_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering navX interface for : " << navX_names_[i] << " at id " << navX_ids_[i]);
		// Create state interface for the given digital input
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::ImuSensorHandle nxsh;
		navX_interface_.registerHandle((nxsh));
		
	}

	// Publish various FRC-specific data using generic joint state for now
	// For simple things this might be OK, but for more complex state
	// (e.g. joystick) it probably makes more sense to write a
	// RealtimePublisher() for the data coming in from
	// the DS
	joint_state_interface_.registerHandle(hardware_interface::JointStateHandle("MatchTime", &match_time_state_, &match_time_state_, &match_time_state_));
	registerInterface(&talon_state_interface_);
	registerInterface(&joint_state_interface_);
	registerInterface(&talon_command_interface_);
	registerInterface(&joint_velocity_interface_);
	registerInterface(&joint_position_interface_);

	ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface Ready.");
}

void FRCRobotInterface::reset()
{
}

void FRCRobotInterface::printState()
{
	// WARNING: THIS IS NOT REALTIME SAFE
	// FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
	ROS_INFO_STREAM_THROTTLE(1,
							 std::endl << "State" <<
							 std::endl << printStateHelper());
}

std::string FRCRobotInterface::printStateHelper()
{
	std::stringstream ss;
	std::cout.precision(15);

	ss << "    CAN ID       position        velocity        effort" << std::endl;
	for (std::size_t i = 0; i < num_can_talon_srxs_; ++i)
	{
		ss << "j" << i << ":    " ;
		ss << talon_state_[i].getCANID() << "\t ";
		ss << std::fixed << talon_state_[i].getPosition() << "\t ";
		ss << std::fixed << talon_state_[i].getSpeed() << "\t ";
		ss << std::fixed << talon_state_[i].getOutputVoltage() << std::endl;
	}
	return ss.str();
}

std::string FRCRobotInterface::printCommandHelper()
{
	std::stringstream ss;
	std::cout.precision(15);
	ss << "    setpoint" << std::endl;
	for (std::size_t i = 0; i < num_can_talon_srxs_; ++i)
		ss << "j" << i << ": " << std::fixed << talon_command_[i].get() << std::endl;
	return ss.str();
}

void FRCRobotInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
	std::string urdf_string;
	urdf_model_ = new urdf::Model();

	// search and wait for robot_description on param server
	while (urdf_string.empty() && ros::ok())
	{
		std::string search_param_name;
		if (nh.searchParam(param_name, search_param_name))
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << search_param_name);
			nh.getParam(search_param_name, urdf_string);
		}
		else
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << param_name);
			nh.getParam(param_name, urdf_string);
		}

		usleep(100000);
	}

	if (!urdf_model_->initString(urdf_string))
		ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
	else
		ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
}

}  // namespace
