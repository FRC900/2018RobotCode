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
	, num_compressors_(0)
	, num_rumble_(0)
	, num_navX_(0)
	, num_analog_inputs_(0)
	, num_dummy_joints_(0)
    , robot_code_ready_(0.0)
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
					throw std::runtime_error("An invalid Nidec brushless joint invert was specified (expecting a boolean).");
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
					throw std::runtime_error("An invalid digital input joint invert was specified (expecting a boolean).");
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
					throw std::runtime_error("An invalid digital output joint invert was specified (expecting a boolean).");
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
					throw std::runtime_error("An invalid pwm joint invert was specified (expecting a boolean).");
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

			if (!joint_params.hasMember("pcm"))
				throw std::runtime_error("A pcm was not specified");
			XmlRpc::XmlRpcValue &xml_solenoid_pcm = joint_params["pcm"];
			if (!xml_solenoid_pcm.valid() ||
					xml_solenoid_pcm.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint solenoid pcm was specified (expecting an int).");

			const int solenoid_pcm = xml_solenoid_pcm;

			solenoid_names_.push_back(joint_name);
			solenoid_ids_.push_back(solenoid_id);
			solenoid_pcms_.push_back(solenoid_pcm);
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

			if (!joint_params.hasMember("pcm"))
				throw std::runtime_error("A pcm was not specified");
			XmlRpc::XmlRpcValue &xml_double_solenoid_pcm = joint_params["pcm"];
			if (!xml_double_solenoid_pcm.valid() ||
					xml_double_solenoid_pcm.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint double solenoid pcm was specified (expecting an int).");

			const int double_solenoid_pcm = xml_double_solenoid_pcm;

			double_solenoid_names_.push_back(joint_name);
			double_solenoid_forward_ids_.push_back(double_solenoid_forward_id);
			double_solenoid_reverse_ids_.push_back(double_solenoid_reverse_id);
			double_solenoid_pcms_.push_back(double_solenoid_pcm);

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
			// TODO : id might instead be a string - MXP, USB, etc
			// telling where the navX is attached?
			if (!joint_params.hasMember("id"))
				throw std::runtime_error("A navX id was not specified");
			XmlRpc::XmlRpcValue &xml_navX_id = joint_params["id"];
			if (!xml_navX_id.valid() ||
					xml_navX_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint id was specified (expecting an int).");

			const int navX_id = xml_navX_id;

			if (!joint_params.hasMember("frame_id"))
				throw std::runtime_error("A navX frame ID was not specified");
			XmlRpc::XmlRpcValue &xml_joint_frame_id= joint_params["frame_id"];
			if (!xml_joint_frame_id.valid() ||
					xml_joint_frame_id.getType() != XmlRpc::XmlRpcValue::TypeString)
				throw std::runtime_error("An invalid navX frame_id was specified (expecting a string).");
			const std::string frame_id = xml_joint_frame_id;

			navX_names_.push_back(joint_name);
			navX_frame_ids_.push_back(frame_id);
			navX_ids_.push_back(navX_id);
		}
		else if (joint_type == "analog_input")
		{
			if (!joint_params.hasMember("analog_channel"))
				throw std::runtime_error("A Analog input analog_channel was not specified");
			XmlRpc::XmlRpcValue &xml_analog_input_analog_channel = joint_params["analog_channel"];
			if (!xml_analog_input_analog_channel.valid() ||
					xml_analog_input_analog_channel.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint analog_channel was specified (expecting an int).");

			const int analog_input_analog_channel = xml_analog_input_analog_channel;

			double analog_input_a;

			if (!joint_params.hasMember("analog_a"))
				analog_input_a = 1;
			else
			{
				XmlRpc::XmlRpcValue &xml_analog_input_a = joint_params["analog_a"];
				if (!xml_analog_input_a.valid() ||
					xml_analog_input_a.getType() != XmlRpc::XmlRpcValue::TypeDouble)
					throw std::runtime_error("An invalid joint a term was specified (expecting an double).");
				analog_input_a = xml_analog_input_a;
			}


			double analog_input_b;
			if (!joint_params.hasMember("analog_b"))
				analog_input_b = 0;
			else
			{
				XmlRpc::XmlRpcValue &xml_analog_input_b = joint_params["analog_b"];
				if (!xml_analog_input_b.valid() ||
					xml_analog_input_b.getType() != XmlRpc::XmlRpcValue::TypeDouble)
					throw std::runtime_error("An invalid joint b term was specified (expecting an double).");
				analog_input_b = xml_analog_input_b;
			}

			analog_input_a_.push_back(analog_input_a);
			analog_input_b_.push_back(analog_input_b);
			analog_input_names_.push_back(joint_name);
			analog_input_analog_channels_.push_back(analog_input_analog_channel);
		}
		else if (joint_type == "compressor")
		{
			if (!joint_params.hasMember("pcm_id"))
				throw std::runtime_error("A compressor pcm id was not specified");
			XmlRpc::XmlRpcValue &xml_compressor_pcm_id = joint_params["pcm_id"];
			if (!xml_compressor_pcm_id.valid() ||
					xml_compressor_pcm_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid compressor joint pcm id was specified (expecting an int).");

			const int compressor_pcm_id = xml_compressor_pcm_id;

			compressor_names_.push_back(joint_name);
			compressor_pcm_ids_.push_back(compressor_pcm_id);
		}
		else if (joint_type == "dummy")
		{
			dummy_joint_names_.push_back(joint_name);
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

		can_talon_srx_run_profile_stop_time_.push_back(0);
		// Do the same for a command interface for
		// the same talon
		hardware_interface::TalonCommandHandle tch(tsh, &talon_command_[i]);
		talon_command_interface_.registerHandle(tch);
	}

	// Set vectors to correct size to hold data
	// for each of the brushless motors we're trying
	// to control
	num_nidec_brushlesses_ = nidec_brushless_names_.size();
	brushless_command_.resize(num_nidec_brushlesses_);
	brushless_vel_.resize(num_nidec_brushlesses_);
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << nidec_brushless_names_[i] << " at PWM channel " << nidec_brushless_pwm_channels_[i] << " / DIO channel " << nidec_brushless_dio_channels_[i]);

		brushless_command_[i] = 0;

		// Create state interface for the given brushless motor
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::JointStateHandle jsh(nidec_brushless_names_[i], &brushless_vel_[i], &brushless_vel_[i], &brushless_vel_[i]);
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
	digital_output_command_.resize(num_digital_outputs_);
	digital_output_state_.resize(num_digital_outputs_);
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		digital_output_state_[i] = std::numeric_limits<double>::max();
		digital_output_command_[i] = 0;

		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << digital_output_names_[i] << " at DIO channel " << digital_output_dio_channels_[i] << " / invert " << digital_output_inverts_[i]);

		hardware_interface::JointStateHandle dosh(digital_output_names_[i], &digital_output_state_[i], &digital_output_state_[i], &digital_output_state_[i]);
		joint_state_interface_.registerHandle(dosh);

		// Do the same for a command interface for
		// the digital output
		hardware_interface::JointHandle doh(dosh, &digital_output_command_[i]);
		joint_position_interface_.registerHandle(doh);
	}

	num_pwm_ = pwm_names_.size();
	pwm_state_.resize(num_pwm_);
	pwm_command_.resize(num_pwm_);
	for (size_t i = 0; i < num_pwm_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << pwm_names_[i] << " at PWM channel " << pwm_pwm_channels_[i] << " / invert " << pwm_inverts_[i]);
		pwm_state_[i] = std::numeric_limits<double>::max();
		pwm_command_[i] = 0;

		hardware_interface::JointStateHandle psh(pwm_names_[i], &pwm_state_[i], &pwm_state_[i], &pwm_state_[i]);
		joint_state_interface_.registerHandle(psh);

		hardware_interface::JointHandle ph(psh, &pwm_command_[i]);
		joint_velocity_interface_.registerHandle(ph);
	}
	num_solenoids_ = solenoid_names_.size();
	solenoid_state_.resize(num_solenoids_);
	solenoid_command_.resize(num_solenoids_);
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << solenoid_names_[i] << " at id " << solenoid_ids_[i]<< " at pcm " << solenoid_pcms_[i]);

		solenoid_state_[i] = std::numeric_limits<double>::max();
		solenoid_command_[i] = 0;

		hardware_interface::JointStateHandle ssh(solenoid_names_[i], &solenoid_state_[i], &solenoid_state_[i], &solenoid_state_[i]);
		joint_state_interface_.registerHandle(ssh);

		hardware_interface::JointHandle soh(ssh, &solenoid_command_[i]);
		joint_position_interface_.registerHandle(soh);
	}

	num_double_solenoids_ = double_solenoid_names_.size();
	double_solenoid_state_.resize(num_double_solenoids_);
	double_solenoid_command_.resize(num_double_solenoids_);
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << double_solenoid_names_[i] << " at forward id " << double_solenoid_forward_ids_[i] << " at reverse id " << double_solenoid_reverse_ids_[i] << " at pcm " << double_solenoid_pcms_[i]);

		double_solenoid_state_[i] = std::numeric_limits<double>::max();
		double_solenoid_command_[i] = 0;

		hardware_interface::JointStateHandle dssh(double_solenoid_names_[i], &double_solenoid_state_[i], &double_solenoid_state_[i], &double_solenoid_state_[i]);
		joint_state_interface_.registerHandle(dssh);

		hardware_interface::JointHandle dsoh(dssh, &double_solenoid_command_[i]);
		joint_position_interface_.registerHandle(dsoh);
	}
	num_rumble_ = rumble_names_.size();
	rumble_state_.resize(num_rumble_);
	rumble_command_.resize(num_rumble_);
	for (size_t i = 0; i < num_rumble_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << rumble_names_[i] << " at port " <<rumble_ports_[i]);

		rumble_state_[i] = std::numeric_limits<double>::max();
		rumble_command_[i] = 0;
		hardware_interface::JointStateHandle rsh(rumble_names_[i], &rumble_state_[i], &rumble_state_[i], &rumble_state_[i]);
		joint_state_interface_.registerHandle(rsh);

		hardware_interface::JointHandle rh(rsh, &rumble_command_[i]);
		joint_position_interface_.registerHandle(rh);
	}

	// Differentiate between navX and IMU here
	// We might want more than 1 type of IMU
	// at some point - eventually allow this by making IMU
	// data sized to hold results from all IMU
	// hardware rather than just navX size
	num_navX_ = navX_names_.size();
	imu_orientations_.resize(num_navX_);
	imu_orientation_covariances_.resize(num_navX_);
	imu_angular_velocities_.resize(num_navX_);
	imu_angular_velocity_covariances_.resize(num_navX_);
	imu_linear_accelerations_.resize(num_navX_);
	imu_linear_acceleration_covariances_.resize(num_navX_);
	navX_state_.resize(num_navX_);
	offset_navX_.resize(num_navX_);

	for (size_t i = 0; i < num_navX_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering navX interface for : " << navX_names_[i] << " at id " << navX_ids_[i]);

		// Create state interface for the given IMU
		// and point it to the data stored in the
		// corresponding imu arrays
		hardware_interface::ImuSensorHandle::Data imu_data;
		imu_data.name = navX_names_[i];
		imu_data.frame_id = navX_frame_ids_[i];
		imu_data.orientation = &imu_orientations_[i][0];
		imu_data.orientation_covariance = &imu_orientation_covariances_[i][0];
		imu_data.angular_velocity = &imu_angular_velocities_[i][0];
		imu_data.angular_velocity_covariance = &imu_angular_velocity_covariances_[i][0];
		imu_data.linear_acceleration = &imu_linear_accelerations_[i][0];
		imu_data.linear_acceleration_covariance = &imu_linear_acceleration_covariances_[i][0];

		hardware_interface::ImuSensorHandle imuh(imu_data);
		imu_interface_.registerHandle(imuh);

		// Set up a command interface to set an
		// offset for reported heading
		hardware_interface::JointStateHandle nxsh(navX_names_[i], &navX_state_[i], &navX_state_[i], &navX_state_[i]);
		joint_state_interface_.registerHandle(nxsh);

		offset_navX_[i] = 0;
	}

	num_analog_inputs_ = analog_input_names_.size();
	analog_input_state_.resize(num_analog_inputs_);
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << analog_input_names_[i] << " at analog channel " << analog_input_analog_channels_[i]);
		// Create state interface for the given analog input
		// and point it to the data stored in the
		// corresponding brushless_state array entry
		hardware_interface::JointStateHandle aish(analog_input_names_[i], &analog_input_state_[i], &analog_input_state_[i], &analog_input_state_[i]);
		joint_state_interface_.registerHandle(aish);
	}
	num_compressors_ = compressor_names_.size();
	compressor_state_.resize(num_compressors_);
	compressor_command_.resize(num_compressors_);
	last_compressor_command_.resize(num_compressors_);
	for (size_t i = 0; i < num_compressors_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for : " << compressor_names_[i] << " at pcm_id " << compressor_pcm_ids_[i]);

		last_compressor_command_[i] = std::numeric_limits<double>::max();
		compressor_command_[i] = 0;
		compressor_state_[i] = 0;

		hardware_interface::JointStateHandle csh(compressor_names_[i], &compressor_state_[i], &compressor_state_[i], &compressor_state_[i]);
		joint_state_interface_.registerHandle(csh);

		hardware_interface::JointHandle cch(csh, &compressor_command_[i]);
		joint_position_interface_.registerHandle(cch);
	}

	num_dummy_joints_ = dummy_joint_names_.size();
	dummy_joint_position_.resize(num_dummy_joints_);
	dummy_joint_velocity_.resize(num_dummy_joints_);
	dummy_joint_effort_.resize(num_dummy_joints_);
	dummy_joint_command_.resize(num_dummy_joints_);
	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "FRCRobotHWInterface: Registering interface for dummy joint : " << dummy_joint_names_[i]);

		dummy_joint_command_[i] = 0;
		dummy_joint_position_[i] = 0;
		dummy_joint_velocity_[i] = 0;
		dummy_joint_effort_[i] = 0;

		hardware_interface::JointStateHandle dsh(dummy_joint_names_[i], &dummy_joint_position_[i],&dummy_joint_velocity_[i], &dummy_joint_effort_[i]);
		joint_state_interface_.registerHandle(dsh);

		hardware_interface::JointHandle dch(dsh, &dummy_joint_command_[i]);
		joint_command_interface_.registerHandle(dch);
		joint_position_interface_.registerHandle(dch);
		joint_velocity_interface_.registerHandle(dch);
	}

	hardware_interface::PDPStateHandle psh("pdp_name", &pdp_state_);
	pdp_state_interface_.registerHandle(psh);

	// Add a flag which indicates we should signal
	// the driver station that robot code is initialized
	hardware_interface::JointStateHandle sh("robot_code_ready", &robot_code_ready_, &robot_code_ready_, &robot_code_ready_);
	joint_state_interface_.registerHandle(sh);

	hardware_interface::JointHandle ch(sh, &robot_code_ready_);
	joint_command_interface_.registerHandle(ch);
	joint_position_interface_.registerHandle(ch);
	joint_velocity_interface_.registerHandle(ch);
	joint_effort_interface_.registerHandle(ch);

	// Publish various FRC-specific data using generic joint state for now
	// For simple things this might be OK, but for more complex state
	// (e.g. joystick) it probably makes more sense to write a
	// RealtimePublisher() for the data coming in from
	// the DS
	registerInterface(&talon_state_interface_);
	registerInterface(&joint_state_interface_);
	registerInterface(&talon_command_interface_);
	registerInterface(&joint_command_interface_);
	registerInterface(&joint_position_interface_);
	registerInterface(&joint_velocity_interface_);
	registerInterface(&joint_effort_interface_); // empty for now
	registerInterface(&imu_interface_);
	registerInterface(&pdp_state_interface_);

	ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface Ready.");
}

// Note - there are two commented-out can_talon calls here.  If
// they're put back in, make customProfileFoo() methods which
// are virtual in the frc_robot_interface def.  For a Set()
// call, make the virtual one do nothing (like the other
// calls already). Then in the hw_interface, override it
// with a method which makes the actual talon call
// For get, not sure what to do exactly?  The hw one is obvious-
// get the value from the talon. For sim, maybe grab it from state?
void FRCRobotInterface::custom_profile_set_talon(hardware_interface::TalonMode mode, double setpoint, double fTerm, int joint_id, int pidSlot, bool zeroPos, double start_run, int &slot_last)
{
	if(zeroPos)
	{
		//pos_offset = can_talons_[joint_id]->GetSelectedSensorPosition(pidIdx) /* radians_scale*/;

		setSensorPosition(joint_id, 0);
		talon_state_[joint_id].setPosition(0);
		ROS_WARN_STREAM("zeroing talon:" <<  joint_id);
	}
	//set talon
	if(mode == hardware_interface::TalonMode_PercentOutput)
	{
		customProfileSetMode(joint_id, mode, setpoint, hardware_interface::DemandType::DemandType_Neutral, 0);
	}
	else
	{	
		customProfileSetMode(joint_id, mode, setpoint, hardware_interface::DemandType::DemandType_ArbitraryFeedForward, fTerm);
		talon_command_[joint_id].setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
		talon_command_[joint_id].setDemand1Value(fTerm);

		hardware_interface::DemandType demand1_type_internal;
		talon_command_[joint_id].demand1Changed(demand1_type_internal, fTerm);

		talon_state_[joint_id].setDemand1Type(demand1_type_internal);
		talon_state_[joint_id].setDemand1Value(fTerm);
	}	

	//ROS_INFO_STREAM("setpoint: " << setpoint << " fterm: " << fTerm << " id: " << joint_id << " offset " << pos_offset << " slot: " << pidSlot << " pos mode? " << posMode);
	
	talon_command_[joint_id].setMode(mode);
	talon_command_[joint_id].newMode(mode);
	talon_state_[joint_id].setTalonMode(mode);

	talon_command_[joint_id].set(setpoint);
	talon_command_[joint_id].commandChanged(setpoint);
	talon_state_[joint_id].setSetpoint(setpoint);

	talon_state_[joint_id].setNeutralOutput(false); // maybe make this a part of setSetpoint?

	talon_command_[joint_id].setPidfSlot(pidSlot);
	// Need a call to PidfSlot changed here?

	if(ros::Time::now().toSec() - start_run < .2 || slot_last != pidSlot)
    {
        ROS_INFO_STREAM("set pid on " << talon_state_[joint_id].getCANID() << "  to: " << pidSlot);

        //can_talons_[joint_id]->SelectProfileSlot(pidSlot, timeoutMs);
        talon_state_[joint_id].setSlot(pidSlot);
    }
    slot_last = pidSlot;
}

void FRCRobotInterface::custom_profile_thread(int joint_id)
{
	//I wonder how inefficient it is to have all of these threads 
	//running at the specified hz just copying to the status

	double time_start = ros::Time::now().toSec();
	int num_slots = 20; //Needs to be the same as the talon command interface and talon state interface
	hardware_interface::CustomProfileStatus status; //Status is also used to store info from last loop
	int points_run = 0;

	std::vector<std::vector<hardware_interface::CustomProfilePoint>> saved_points;
	saved_points.resize(num_slots);

	std::vector<std::vector<double>> saved_times;
	saved_times.resize(num_slots);

	int slot_last = -1;

	while (ros::ok())
	{
		if (talon_state_[joint_id].getTalonMode() == hardware_interface::TalonMode_Follower)
		{
			ROS_INFO("Exiting custom_profile_thread since mode == Follower");
			return;
		}

		talon_command_[joint_id].getCustomProfilePointsTimesChanged(saved_points, saved_times);

		ros::Rate rate(talon_command_[joint_id].getCustomProfileHz());
		bool run = talon_command_[joint_id].getCustomProfileRun();

		if(status.running && !run)
		{		
			std::vector<hardware_interface::CustomProfilePoint> empty_points;
			talon_command_[joint_id].overwriteCustomProfilePoints(empty_points, status.slotRunning);	
			//Right now we wipe everything if the profile is stopped
			//This could be changed to a pause type feature in which the first point has zeroPos set and the other
			//positions get shifted
			points_run = 0;
		}
		if((run && !status.running) || !run) 
		{
			time_start = ros::Time::now().toSec();
		}
		int slot = talon_command_[joint_id].getCustomProfileSlot();

		if(slot != status.slotRunning && run && status.running)
		{
			ROS_WARN("transitioned between two profile slots without any break between. Intended?");
			std::vector<hardware_interface::CustomProfilePoint> empty_points;
			talon_command_[joint_id].overwriteCustomProfilePoints(empty_points, status.slotRunning);	
			//Right now we wipe everything if the slots are flipped
			//Should try to be analagous to having a break between
			points_run = 0;
			time_start = ros::Time::now().toSec();
		}
		status.slotRunning = slot;	
		if(run)
		{
			if(saved_points[slot].size() == 0)
			{
				static int fail_flag = 0;
				if(fail_flag % 100 == 0)
				{
					ROS_ERROR("Tried to run custom profile with no points buffered");
				}
				//Potentially add more things to do if this exception is caught
				//Like maybe set talon to neutral mode or something
				fail_flag++;
				continue;
			}


			//TODO below isn't copying correct?

			int start = points_run - 1;
			if(start < 0) start = 0;
			int end;
			status.outOfPoints = true;
			double time_since_start = ros::Time::now().toSec() - time_start;
			for(; start < saved_points[slot].size(); start++)
			{
				//Find the point just greater than time since start	
				if(saved_times[slot][start] > time_since_start)
				{
					status.outOfPoints = false;
					end = start;
					break;
				}
			}
			if(status.outOfPoints)
			{
				points_run = saved_points[slot].size();
			}
			else
			{
				points_run = end -1;	
				if(points_run < 0) points_run = 0;
			}
			if(status.outOfPoints)
			{
				auto next_slot = talon_command_[joint_id].getCustomProfileNextSlot();

				//If all points have been exhausted, just use the last point
				custom_profile_set_talon(saved_points[slot].back().mode, saved_points[slot].back().setpoint, saved_points[slot].back().fTerm, joint_id, saved_points[slot].back().pidSlot, saved_points[slot].back().zeroPos, time_start, slot_last);
				if((next_slot.size() > 0))
				{
					talon_command_[joint_id].setCustomProfileSlot(next_slot[0]);
					next_slot.erase(next_slot.begin());
					talon_command_[joint_id].setCustomProfileNextSlot(next_slot);
				}
			}
			else if(end == 0)
			{
				//If we are still on the first point,just use the first point
				custom_profile_set_talon(saved_points[slot][0].mode, saved_points[slot][0].setpoint, saved_points[slot][0].fTerm, joint_id, saved_points[slot][0].pidSlot, saved_points[slot][0].zeroPos,  time_start, slot_last);
			}
			else
			{
				//Allows for mode flipping while in profile execution
				//We don't want to interpolate between positional and velocity setpoints
				if(saved_points[slot][end].mode != saved_points[slot][end-1].mode)
				{
					ROS_WARN("mid profile mode flip. If intended, Cooooooooollllll. If not, fix the code");
					custom_profile_set_talon(saved_points[slot][end].mode, saved_points[slot][end].setpoint, saved_points[slot][end].fTerm, joint_id, saved_points[slot][end].pidSlot, saved_points[slot][end].zeroPos,  time_start, slot_last);
					// consider adding a check to see which is closer
				}
				else
				{
					//linear interpolation
					double setpoint = saved_points[slot][end - 1].setpoint + (saved_points[slot][end].setpoint - saved_points[slot][end - 1].setpoint) / 
						(saved_times[slot][end] - saved_times[slot][end-1]) * (time_since_start - saved_times[slot][end-1]);


					double fTerm = saved_points[slot][end - 1].fTerm + (saved_points[slot][end].fTerm - saved_points[slot][end - 1].fTerm) / 
						(saved_times[slot][end] - saved_times[slot][end-1]) * (time_since_start - saved_times[slot][end-1]);

					custom_profile_set_talon(saved_points[slot][end].mode, setpoint, fTerm, joint_id, saved_points[slot][end].pidSlot, saved_points[slot][end-1].zeroPos, time_start, slot_last);

				}
			}
		}
		else
		{
			status.outOfPoints = false;
		}
		for(int i = 0; i < num_slots; i++)
		{
			if(i == status.slotRunning)
			{
				status.remainingPoints[i] = talon_command_[joint_id].getCustomProfileCount(i) - points_run;
				if(talon_command_[joint_id].getCustomProfileTimeCount(i) != 0)
				{
					status.remainingTime = talon_command_[joint_id].getCustomProfileEndTime(i) - (ros::Time::now().toSec() - time_start);
				}
				else
				{
					status.remainingTime = 0.0;
				}
			}
			else
			{
				status.remainingPoints[i] = talon_command_[joint_id].getCustomProfileCount(i);
			}
		}

		status.running = run;
		talon_state_[joint_id].setCustomProfileStatus(status);
		rate.sleep();
	}
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
	return;
#if 0
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
#endif
}

}  // namespace
