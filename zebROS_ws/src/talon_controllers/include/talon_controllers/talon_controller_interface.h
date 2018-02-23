#pragma once
#include <dynamic_reconfigure/server.h>
#include <talon_interface/talon_command_interface.h>
#include <talon_controllers/TalonConfigConfig.h>
#include <XmlRpcValue.h>

namespace talon_controllers
{
// Create a wrapper class for each Talon mode.  For the basic controllers
// this isn't really helpful since the code could just be included in the
// controller itself.  But consider a more complex controller, for example
// swerve. The swerve controller runs a number of wheels, and each wheel
// has both position and velocity.  The wheel class might create a
// TalonPosisionPIDControllerInterface member var for the position moter and
// also a TalonVelocityPIDControllerInterface member var for the velocity.
// And since it will be creating one per wheel, it makes sense to wrap the common
// init code into a class rather than duplicate it for each wheel. Another
// controller - say a shooter wheel - could also use this same code to
// create a talon handle to access that motor
//

// Class which provides a common set of code for reading
// parameters for motor controllers from yaml / command line
// ROS params.  Not all of these values will be needed for all
// modes - specific controller interfaces will use what
// they do need and ignore the rest.
// The idea here is that code using a particular CI would
// call readParams(), modify any parameters which are specific
// to the controller, and then call init using the specificed
// parameters. This will handle the common case where most
// code using the CI will want to use the default names of settings
// but also allow customization
class TalonCIParams
{
	public:
		// Initialize with relatively sane defaults
		// for all parameters
		TalonCIParams(void) :
			p_{0, 0},
			i_{0, 0},
			d_{0, 0},
			f_{0, 0},
			izone_{0, 0},
			allowable_closed_loop_error_{0, 0}, // need better defaults
			max_integral_accumulator_{0, 0},
			pidf_slot_(0),
			invert_output_(false),
			sensor_phase_(false),
			neutral_mode_(hardware_interface::NeutralMode_Uninitialized),
			feedback_type_(hardware_interface::FeedbackDevice_Uninitialized),
			ticks_per_rotation_(4096),
			closed_loop_ramp_(0.),
			open_loop_ramp_(0.),
			peak_output_forward_(1.),
			peak_output_reverse_(-1.),
			nominal_output_forward_(0.),
			nominal_output_reverse_(0.),
			neutral_deadband_(0.),
			voltage_compensation_saturation_(12.5),
			voltage_measurement_filter_(32),
			voltage_compensation_enable_(true),
			limit_switch_local_forward_source_(hardware_interface::LimitSwitchSource_FeedbackConnector),
			limit_switch_local_forward_normal_(hardware_interface::LimitSwitchNormal_NormallyOpen),
			limit_switch_local_reverse_source_(hardware_interface::LimitSwitchSource_FeedbackConnector),
			limit_switch_local_reverse_normal_(hardware_interface::LimitSwitchNormal_NormallyOpen),
			softlimit_forward_threshold_(0.0),
			softlimit_forward_enable_(false),
			softlimit_reverse_threshold_(0.0),
			softlimit_reverse_enable_(false),
			softlimits_override_enable_(true),
			current_limit_peak_amps_(0),
			current_limit_peak_msec_(0),
			current_limit_continuous_amps_(0),
			current_limit_enable_(false),
			motion_cruise_velocity_(0), // No idea at a guess
			motion_acceleration_(0),
			motion_control_frame_period_(20), // Guess at 50Hz default?
			
			conversion_factor_(1.0)
		{
		}

		// Update params set by a dynamic reconfig config
		// Also pass in current params for ones which aren't
		// dynamically reconfigurable - pass them through
		// to the new one 
		TalonCIParams(const TalonConfigConfig &config)
		{
			p_[0] = config.p0;
			p_[1] = config.p1;
			i_[0] = config.i0;
			i_[1] = config.i1;
			d_[0] = config.d0;
			d_[1] = config.d1;
			f_[0] = config.f0;
			f_[1] = config.f1;
			izone_[0] = config.izone0;
			izone_[1] = config.izone1;
			allowable_closed_loop_error_[0] = config.allowable_closed_loop_error0;
			allowable_closed_loop_error_[1] = config.allowable_closed_loop_error1;
			max_integral_accumulator_[0] = config.max_integral_accumulator0;
			max_integral_accumulator_[1] = config.max_integral_accumulator1;
			pidf_slot_ = config.pid_config;
			invert_output_ = config.invert_output;

			sensor_phase_ = config.sensor_phase;
			feedback_type_ = static_cast<hardware_interface::FeedbackDevice>(config.feedback_type);
			ticks_per_rotation_ = config.encoder_ticks_per_rotation;
			neutral_mode_ = static_cast<hardware_interface::NeutralMode>(config.neutral_mode);
			closed_loop_ramp_ = config.closed_loop_ramp;
			open_loop_ramp_ = config.open_loop_ramp;
			peak_output_forward_ = config.peak_output_forward;
			peak_output_reverse_ = config.peak_output_reverse;
			nominal_output_forward_ = config.nominal_output_forward;
			nominal_output_reverse_ = config.nominal_output_reverse;
			neutral_deadband_ = config.neutral_deadband;
			voltage_compensation_saturation_ = config.voltage_compensation_saturation;
			voltage_measurement_filter_ = config.voltage_measurement_filter;
			voltage_compensation_enable_ = config.voltage_compensation_enable;
			limit_switch_local_forward_source_ = static_cast<hardware_interface::LimitSwitchSource>(config.limit_switch_local_forward_source);
			limit_switch_local_forward_normal_ = static_cast<hardware_interface::LimitSwitchNormal>(config.limit_switch_local_forward_normal);
			limit_switch_local_reverse_source_ = static_cast<hardware_interface::LimitSwitchSource>(config.limit_switch_local_reverse_source);
			limit_switch_local_reverse_normal_ = static_cast<hardware_interface::LimitSwitchNormal>(config.limit_switch_local_reverse_normal);

			softlimit_forward_threshold_ = config.softlimit_forward_threshold;
			softlimit_forward_enable_ = config.softlimit_forward_enable;
			softlimit_reverse_threshold_ = config.softlimit_reverse_threshold;
			softlimit_reverse_enable_ = config.softlimit_reverse_enable;
			softlimits_override_enable_ = config.softlimits_override_enable;

			current_limit_peak_amps_ = config.current_limit_peak_amps;
			current_limit_peak_msec_ = config.current_limit_peak_msec;
			current_limit_continuous_amps_ = config.current_limit_continuous_amps;
			current_limit_enable_ = config.current_limit_enable;
			motion_cruise_velocity_ = config.motion_cruise_velocity;
			motion_acceleration_ = config.motion_acceleration;
			motion_control_frame_period_ = config.motion_control_frame_period;
		
			conversion_factor_ = config.conversion_factor;
		}

		// Copy from internal state to TalonConfigConfig state
		TalonConfigConfig toConfig(void) const
		{
			TalonConfigConfig config;
			config.p0            = p_[0];
			config.p1            = p_[1];
			config.i0            = i_[0];
			config.i1            = i_[1];
			config.d0            = d_[0];
			config.d1            = d_[1];
			config.f0            = f_[0];
			config.f1            = f_[1];
			config.izone0        = izone_[0];
			config.izone1        = izone_[1];
			config.allowable_closed_loop_error0 = allowable_closed_loop_error_[0];
			config.allowable_closed_loop_error1 = allowable_closed_loop_error_[1];
			config.max_integral_accumulator0 = max_integral_accumulator_[0];
			config.max_integral_accumulator1 = max_integral_accumulator_[1];
			config.pid_config    = pidf_slot_;
			config.invert_output = invert_output_;
			config.sensor_phase  = sensor_phase_;
			config.feedback_type = feedback_type_;
			config.encoder_ticks_per_rotation = ticks_per_rotation_;
			config.neutral_mode  = neutral_mode_;
			config.closed_loop_ramp = closed_loop_ramp_;
			config.open_loop_ramp = open_loop_ramp_;
			config.peak_output_forward = peak_output_forward_;
			config.peak_output_reverse = peak_output_reverse_;
			config.nominal_output_forward = nominal_output_forward_;
			config.nominal_output_reverse = nominal_output_reverse_;
			config.neutral_deadband = neutral_deadband_;
			config.voltage_compensation_saturation = voltage_compensation_saturation_;
			config.voltage_measurement_filter = voltage_measurement_filter_;
			config.voltage_compensation_enable = voltage_compensation_enable_;
			config.limit_switch_local_forward_source = limit_switch_local_forward_source_;
			config.limit_switch_local_forward_normal = limit_switch_local_forward_normal_;
			config.limit_switch_local_reverse_source = limit_switch_local_reverse_source_;
			config.limit_switch_local_reverse_normal = limit_switch_local_reverse_normal_;
			config.softlimit_forward_threshold = softlimit_forward_threshold_;
			config.softlimit_forward_enable = softlimit_forward_enable_;
			config.softlimit_reverse_threshold = softlimit_reverse_threshold_;
			config.softlimit_reverse_enable = softlimit_reverse_enable_;
			config.softlimits_override_enable = softlimits_override_enable_;
			config.current_limit_peak_amps = current_limit_peak_amps_;
			config.current_limit_peak_msec = current_limit_peak_msec_;
			config.current_limit_continuous_amps = current_limit_continuous_amps_;
			config.current_limit_enable = current_limit_enable_;
			config.motion_cruise_velocity = motion_cruise_velocity_;
			config.motion_acceleration = motion_acceleration_;
			config.motion_control_frame_period = motion_control_frame_period_;
			config.conversion_factor = conversion_factor_;
			return config;
		}

		// Read a joint name from the given nodehandle's params
		bool readJointName(ros::NodeHandle &n)
		{
			if (!n.getParam("joint", joint_name_))
			{
				ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
				return false;
			}
			return true;
		}

		bool readConversion(ros::NodeHandle &n)
		{
			n.getParam("conversion_factor", conversion_factor_);
			return true;
		}

		// Read a joint name from the given nodehandle's params
		bool readNeutralMode(ros::NodeHandle &n)
		{
			std::string mode_string;
			if (n.getParam("neutral_mode", mode_string))
			{
				if (mode_string == "EEPROM")
					neutral_mode_ = hardware_interface::NeutralMode_EEPROM_Setting;
				else if (mode_string == "Coast")
					neutral_mode_ = hardware_interface::NeutralMode_Coast;
				else if (mode_string == "Brake")
					neutral_mode_ = hardware_interface::NeutralMode_Brake;
				else
				{
					ROS_ERROR("Invalid neutral mode name (namespace: %s, %s)",
							  n.getNamespace().c_str(), mode_string.c_str());
					return false;
				}
			}
			return true;
		}
		//TODOa: create a method that reads the feedback settings enum
		bool readFeedbackType(ros::NodeHandle &n)
		{
			std::string feedback_type_name;
			if (!n.getParam("feedback_type", feedback_type_name))
			{
				//ROS_ERROR("No feedback type given (namespace: %s)",
				//			  n.getNamespace().c_str());
				// TODO : Not all talons will have feedback - figure
				//        out how to handle that case
				return true;
			}
			if (feedback_type_name == "QuadEncoder")
				feedback_type_ = hardware_interface::FeedbackDevice_QuadEncoder;
			else if (feedback_type_name == "Analog")
				feedback_type_ = hardware_interface::FeedbackDevice_Analog;
			else if (feedback_type_name == "Tachometer")
				feedback_type_ = hardware_interface::FeedbackDevice_Tachometer;
			else if (feedback_type_name == "PulseWidthEncodedPosition")
				feedback_type_ = hardware_interface::FeedbackDevice_PulseWidthEncodedPosition;
			else if (feedback_type_name == "SensorSum")
				feedback_type_ = hardware_interface::FeedbackDevice_SensorSum;
			else if (feedback_type_name == "SensorDifference")
				feedback_type_ = hardware_interface::FeedbackDevice_SensorDifference;
			else if (feedback_type_name == "RemoteSensor0")
				feedback_type_ = hardware_interface::FeedbackDevice_RemoteSensor0;
			else if (feedback_type_name == "RemoteSensor1")
				feedback_type_ = hardware_interface::FeedbackDevice_RemoteSensor1;
			else if (feedback_type_name == "SoftwareEmulatedSensor")
				feedback_type_ = hardware_interface::FeedbackDevice_SoftwareEmulatedSensor;
			else if (feedback_type_name == "CTRE_MagEncoder_Absolute")
				feedback_type_ = hardware_interface::FeedbackDevice_CTRE_MagEncoder_Absolute;
			else if (feedback_type_name == "CTRE_MagEncoder_Relative")
				feedback_type_ = hardware_interface::FeedbackDevice_CTRE_MagEncoder_Relative;
			else
			{
				ROS_ERROR("Invalid feedback device name given");
				return false;
			}
			n.getParam("ticks_per_rotation", ticks_per_rotation_);
			return true;
		}

		bool readInverts(ros::NodeHandle &n)
		{
			n.getParam("invert_output", invert_output_);
			n.getParam("sensor_phase", sensor_phase_);
			return true;
		}

		bool readCloseLoopParams(ros::NodeHandle &n)
		{
			XmlRpc::XmlRpcValue pid_param_list;

			if (!n.getParam("close_loop_values", pid_param_list))
				return true;
			if (pid_param_list.size() <= 2)
			{
				for (int i = 0; i < pid_param_list.size(); i++)
				{
					XmlRpc::XmlRpcValue &pidparams_ = pid_param_list[i];

					p_[i] = findFloatParam("p", pidparams_);
					i_[i] = findFloatParam("i", pidparams_);
					d_[i] = findFloatParam("d", pidparams_);
					f_[i] = findFloatParam("f", pidparams_);
					izone_[i] = findIntParam("i_zone", pidparams_);
					allowable_closed_loop_error_[i] = findIntParam("allowable_closed_loop_error", pidparams_);
					max_integral_accumulator_[i] = findFloatParam("max_integral_accumulator", pidparams_);
				}
				return true;
			}
			else
			{
				throw std::runtime_error("More than two pid_param values");
			}
			return false;
		}

		bool readOutputShaping(ros::NodeHandle &n)
		{
			n.getParam("closed_loop_ramp", closed_loop_ramp_);
			n.getParam("open_loop_ramp", open_loop_ramp_);
			n.getParam("peak_output_forward", peak_output_forward_);
			n.getParam("peak_output_reverse", peak_output_reverse_);
			n.getParam("nominal_output_forward", nominal_output_forward_);
			n.getParam("nominal_output_reverse", nominal_output_reverse_);
			n.getParam("neutral_deadband", neutral_deadband_);
			return true;
		}
		bool readVoltageCompensation(ros::NodeHandle &n)
		{
			int params_read = 0;
			if (n.getParam("voltage_compensation_saturation", voltage_compensation_saturation_))
				params_read += 1;
			if (n.getParam("voltage_measurement_filter", voltage_measurement_filter_))
				params_read += 1;
			if (n.getParam("voltage_compensation_enable", voltage_compensation_enable_) &&
				voltage_compensation_enable_ && (params_read < 2))
				ROS_WARN("Not all voltage compensation params set before enabling - using defaults might not work as expected");
			return true;
		}

		bool readLimitSwitches(ros::NodeHandle &n)
		{
			std::string str_val;
			hardware_interface::LimitSwitchSource limit_switch_source;
			hardware_interface::LimitSwitchNormal limit_switch_normal;
			if (n.getParam("limit_switch_local_forward_source", str_val))
			{
				if (!stringToLimitSwitchSource(str_val, limit_switch_source))
					return false;
				limit_switch_local_forward_source_ = limit_switch_source;
			}
			if (n.getParam("limit_switch_local_forward_normal", str_val))
			{
				if (!stringToLimitSwitchNormal(str_val, limit_switch_normal))
					return false;
				limit_switch_local_forward_normal_ = limit_switch_normal;
			}
			if (n.getParam("limit_switch_local_reverse_source", str_val))
			{
				if (!stringToLimitSwitchSource(str_val, limit_switch_source))
					return false;
				limit_switch_local_reverse_source_ = limit_switch_source;
			}
			if (n.getParam("limit_switch_local_reverse_normal", str_val))
			{
				if (!stringToLimitSwitchNormal(str_val, limit_switch_normal))
					return false;
				limit_switch_local_reverse_normal_ = limit_switch_normal;
			}
			return true;
		}

		bool readSoftLimits(ros::NodeHandle &n)
		{
			int param_count = 0;
			if (n.getParam("softlimit_forward_threshold", softlimit_forward_threshold_))
				param_count = 1;
			if (n.getParam("softlimit_forward_enable", softlimit_forward_enable_) &&
					softlimit_forward_enable_ && (param_count == 0))
				ROS_WARN("Enabling forward softlimits without setting threshold");
			param_count = 0;
			if (n.getParam("softlimit_reverse_threshold", softlimit_reverse_threshold_))
				param_count = 1;
			if (n.getParam("softlimit_reverse_enable", softlimit_reverse_enable_) &&
				softlimit_reverse_enable_ && (param_count == 0))
					ROS_WARN("Enabling forward softlimits without setting threshold");
			return true;
		}

		bool readCurrentLimits(ros::NodeHandle &n)
		{
			int params_read = 0;
			if (n.getParam("current_limit_peak_amps", current_limit_peak_amps_))
				params_read += 1;
			if (n.getParam("current_limit_peak_msec", current_limit_peak_msec_))
				params_read += 1;
			if (n.getParam("current_limit_continuous_amps", current_limit_continuous_amps_))
				params_read += 1;
			if (n.getParam("current_limit_enable", current_limit_enable_) &&
				current_limit_enable_ && (params_read < 3))
				ROS_WARN("Not all current limits set before enabling - using defaults might not work as expected");
			return true;
		}

		bool readMotionControl(ros::NodeHandle &n)
		{
			n.getParam("motion_cruise_velocity", motion_cruise_velocity_);
			n.getParam("motion_acceleration", motion_acceleration_);
			n.getParam("motion_control_frame_period", motion_control_frame_period_);
			return true;
		}

		// TODO : Keep adding config items here
		std::string joint_name_;
		double p_[2];
		double i_[2];
		double d_[2];
		double f_[2];
		int    izone_[2];
		int    allowable_closed_loop_error_[2];
		double max_integral_accumulator_[2];
		int    pidf_slot_;
		bool   invert_output_;
		bool   sensor_phase_;
		hardware_interface::NeutralMode neutral_mode_;
		hardware_interface::FeedbackDevice feedback_type_;
		int    ticks_per_rotation_;
		double closed_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;
		double voltage_compensation_saturation_;
		int    voltage_measurement_filter_;
		bool   voltage_compensation_enable_;

		hardware_interface::LimitSwitchSource limit_switch_local_forward_source_;
		hardware_interface::LimitSwitchNormal limit_switch_local_forward_normal_;
		hardware_interface::LimitSwitchSource limit_switch_local_reverse_source_;
		hardware_interface::LimitSwitchNormal limit_switch_local_reverse_normal_;

		double softlimit_forward_threshold_;
		bool   softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool   softlimit_reverse_enable_;
		bool   softlimits_override_enable_;
		int    current_limit_peak_amps_;
		int    current_limit_peak_msec_;
		int    current_limit_continuous_amps_;
		bool   current_limit_enable_;
		double motion_cruise_velocity_;
		double motion_acceleration_;
		int    motion_control_frame_period_;
		
		double conversion_factor_;
	private:
		// Read a double named <param_type> from the array/map
		// in params
		double findFloatParam(std::string param_type, XmlRpc::XmlRpcValue &params) const
		{
			if (!params.hasMember(param_type))
				return 0;
			XmlRpc::XmlRpcValue &param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a double valid type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
				return (double)param;
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
				return (int)param;
			else
				throw std::runtime_error("A non-double value was passed for" + param_type);
			return 0;
		}

		// Read an integer named <param_type> from the array/map
		// in params
		int findIntParam(std::string param_type, XmlRpc::XmlRpcValue &params) const
		{
			if (!params.hasMember(param_type))
				return 0;
			XmlRpc::XmlRpcValue &param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a valid int type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
				return (int)param;
			else
				throw std::runtime_error("A non-int value was passed for" + param_type);
			return 0;
		}

		bool stringToLimitSwitchSource(const std::string &str,
									   hardware_interface::LimitSwitchSource &limit_switch_source)
		{
			if (str == "FeedbackConnector")
				limit_switch_source = hardware_interface::LimitSwitchSource_FeedbackConnector;
			else if (str == "RemoteTalonSRX")
				limit_switch_source = hardware_interface::LimitSwitchSource_RemoteTalonSRX;
			else if (str == "RemoteCANifier")
				limit_switch_source = hardware_interface::LimitSwitchSource_RemoteCANifier;
			else if (str == "Deactivated")
				limit_switch_source = hardware_interface::LimitSwitchSource_Deactivated;
			else
			{
				ROS_ERROR_STREAM("Invalid limit switch source : " << str);
				return false;
			}
			return true;
		}
		bool stringToLimitSwitchNormal(const std::string &str,
									   hardware_interface::LimitSwitchNormal &limit_switch_source)
		{
			if (str == "NormallyOpen")
				limit_switch_source = hardware_interface::LimitSwitchNormal_NormallyOpen;
			else if (str == "NormallyClosed")
				limit_switch_source = hardware_interface::LimitSwitchNormal_NormallyClosed;
			else if (str == "Disabled")
				limit_switch_source = hardware_interface::LimitSwitchNormal_Disabled;
			else
			{
				ROS_ERROR_STREAM("Invalid limit switch normal : " << str);
				return false;
			}
			return true;
		}
};

// Base class for controller interface types. This class
// will be the least restrictive - allow users to swtich modes,
// reprogram any config values, and so on.
// Derived classes will be more specialized - they'll only allow
// a specific Talon mode and disable code which doesn't apply
// to that mode
class TalonControllerInterface
{
	public:
		TalonControllerInterface(void) :
			srv_(nullptr), 
			srv_mutex_(nullptr)
		{
		}

		// Standardize format for reading params for
		// motor controller
		virtual bool readParams(ros::NodeHandle &n, TalonCIParams &params)
		{
			return params.readJointName(n) &&
				   params.readConversion(n) &&
				   params.readCloseLoopParams(n) &&
				   params.readNeutralMode(n) &&
				   params.readInverts(n) &&
				   params.readFeedbackType(n) &&
				   params.readOutputShaping(n) &&
				   params.readVoltageCompensation(n) &&
				   params.readLimitSwitches(n) &&
				   params.readSoftLimits(n) &&
				   params.readCurrentLimits(n) &&
				   params.readMotionControl(n);
		}

		// Read params from config file and use them to
		// initialize the Talon hardware
		// Useful for the hopefully common case where there's
		// no need to modify the parameters after reading
		// them
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface * /*tsi*/,
								  ros::NodeHandle &n,
								  bool dynamic_reconfigure = false)
		{
			return init(tci, n, talon_, srv_mutex_, srv_, true, dynamic_reconfigure) && 
				   setInitialMode();
		}

		// Same as above, except pass in an array
		// of node handles. First entry is set up as the master
		// talon and the rest are set in follower mode to follow
		// the leader
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface *tsi,
								  std::vector<ros::NodeHandle> &n,
								  bool dynamic_reconfigure = false)
		{
			if (!initWithNode(tci, tsi, n[0], dynamic_reconfigure))
				return false;

			const int follow_can_id = talon_.state()->getCANID();

			follower_talons_.resize(n.size() - 1);
			for (size_t i = 1; i < n.size(); i++)
			{
				follower_srv_mutexes_.push_back(nullptr);
				follower_srvs_.push_back(nullptr);
				if (!init(tci, n[i], follower_talons_[i-1], follower_srv_mutexes_[i-1], follower_srvs_[i-1], false, dynamic_reconfigure))
					return false;
				follower_talons_[i-1]->setMode(hardware_interface::TalonMode_Follower);
				follower_talons_[i-1]->set(follow_can_id);
				ROS_INFO_STREAM("Set up talon " << follower_talons_[i-1].getName() << " to follow CAN ID " << follow_can_id << " (" << talon_.getName() << ")");
			}

			return true;
		}

		// Init with XmlRpcValue instead of NodeHandle - XmlRpcValue
		// will be either a string or an array of strings of joints
		// to load
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface *tsi,
								  ros::NodeHandle &controller_nh,
								  XmlRpc::XmlRpcValue param,
								  bool dynamic_reconfigure = false)
		{
			std::vector<ros::NodeHandle> joint_nodes;

			if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
			{
				if (param.size() == 0)
				{
					ROS_ERROR_STREAM("Joint param is an empty list");
					return false;
				}

				for (int i = 0; i < param.size(); ++i)
				{
					if (param[i].getType() != XmlRpc::XmlRpcValue::TypeString)
					{
						ROS_ERROR_STREAM("Joint param #" << i << " isn't a string.");
						return false;
					}
				}

				for (int i = 0; i < param.size(); ++i)
					joint_nodes.push_back(ros::NodeHandle(controller_nh,
								static_cast<std::string>(param[i])));
			}
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
			{
				joint_nodes.push_back(ros::NodeHandle(controller_nh,
							static_cast<std::string>(param)));
			}
			else
			{
				ROS_ERROR_STREAM("Joint param is neither a list of strings nor a string.");
				return false;
			}

			return initWithNode(tci, tsi, joint_nodes, dynamic_reconfigure);
		}

#if 0
		// Allow users of the ControllerInterface to get
		// a copy of the parameters currently set for the
		// Talon.  They can then modify them at will and
		// call initWithParams to reprogram the Talon.
		// Hopefully this won't be needed all that often ...
		// the goal should be to provide a simpler interface
		// for commonly used operations
		virtual TalonCIParams getParams(void) const
		{
			return params_;
		}

		// Initialize Talon hardware with the settings in params
		virtual bool initWithParams(hardware_interface::TalonCommandInterface *tci,
									const TalonCIParams &params)
		{
			talon_ = tci->getHandle(params.joint_name_);
			return writeParamsToHW(params);
		}
#endif

		// Use data in params to actually set up Talon
		// hardware. Make this a separate method outside of
		// init() so that dynamic reconfigure callback can write
		// values using this method at any time
		virtual bool writeParamsToHW(const TalonCIParams &params, bool update_params = true)
		{
			// perform additional hardware init here
			// but don't set mode - either force the caller to
			// set it or use one of the derived, fixed-mode
			// classes instead
			for (int i = 0; i < 2; i++)
			{
				talon_->setP(params.p_[i], i);
				talon_->setI(params.i_[i], i);
				talon_->setD(params.d_[i], i);
				talon_->setF(params.f_[i], i);
				talon_->setIZ(params.izone_[i], i);
				// TODO : I'm worried about these. We need
				// better default values than 0.0
				talon_->setAllowableClosedloopError(params.allowable_closed_loop_error_[i], i);
				talon_->setMaxIntegralAccumulator(params.max_integral_accumulator_[i], i);
			}
			talon_->setPidfSlot(params.pidf_slot_);
			talon_->setNeutralMode(params.neutral_mode_);

			talon_->setEncoderFeedback(params.feedback_type_);
			talon_->setEncoderTicksPerRotation(params.ticks_per_rotation_);

			talon_->setInvert(params.invert_output_);
			talon_->setSensorPhase(params.sensor_phase_);

			talon_->setClosedloopRamp(params.closed_loop_ramp_);
			talon_->setOpenloopRamp(params.open_loop_ramp_);
			talon_->setPeakOutputForward(params.peak_output_forward_);
			talon_->setPeakOutputReverse(params.peak_output_reverse_);
			talon_->setNominalOutputForward(params.nominal_output_forward_);
			talon_->setNominalOutputReverse(params.nominal_output_reverse_);
			talon_->setNeutralDeadband(params.neutral_deadband_);

			talon_->setVoltageCompensationSaturation(params.voltage_compensation_saturation_);
			talon_->setVoltageMeasurementFilter(params.voltage_measurement_filter_);
			talon_->setVoltageCompensationEnable(params.voltage_compensation_enable_);

			talon_->setForwardLimitSwitchSource(params.limit_switch_local_forward_source_, params.limit_switch_local_forward_normal_);
			talon_->setReverseLimitSwitchSource(params.limit_switch_local_reverse_source_, params.limit_switch_local_reverse_normal_);
			talon_->setOverrideSoftLimitsEnable(params.softlimits_override_enable_);
			talon_->setForwardSoftLimitThreshold(params.softlimit_forward_threshold_);
			talon_->setForwardSoftLimitEnable(params.softlimit_forward_enable_);
			talon_->setReverseSoftLimitThreshold(params.softlimit_reverse_threshold_);
			talon_->setReverseSoftLimitEnable(params.softlimit_reverse_enable_);

			talon_->setPeakCurrentLimit(params.current_limit_peak_amps_);
			talon_->setPeakCurrentDuration(params.current_limit_peak_msec_);
			talon_->setContinuousCurrentLimit(params.current_limit_continuous_amps_);
			talon_->setCurrentLimitEnable(params.current_limit_enable_);

			talon_->setMotionCruiseVelocity(params.motion_cruise_velocity_);
			talon_->setMotionAcceleration(params.motion_acceleration_);
			talon_->setMotionControlFramePeriod(params.motion_control_frame_period_);

			talon_->setConversionFactor(params.conversion_factor_);

			// Save copy of params written to HW
			// so they can be queried later?
			if (update_params)
				params_ = params;

			return true;
		}

		void callback(talon_controllers::TalonConfigConfig &config, uint32_t /*level*/)
		{
			// TODO : this list is rapidly getting out of date.
			// Update it or remove the printout?
			ROS_INFO("Reconfigure request : %s %f %f %f %f %f %f %f %f %f %f %d %d",
					 talon_.getName().c_str(),
					 config.p0,
					 config.p1,
					 config.i0,
					 config.i1,
					 config.d0,
					 config.d1,
					 config.f0,
					 config.f1,
					 config.izone0,
					 config.izone1,
					 config.invert_output,
					 config.sensor_phase);

			writeParamsToHW(TalonCIParams(config));
		}

		// Set the setpoint for the motor controller
		virtual void setCommand(const double command)
		{
			talon_->set(command);
		}
		

		// Set the mode of the motor controller
		virtual void setMode(hardware_interface::TalonMode mode)
		{
			talon_->setMode(mode);
		}

		// Pick the config slot (0 or 1) for PIDF/IZone values
		virtual bool setPIDFSlot(int slot)
		{
			if ((slot != 0) && (slot != 1))
				return false;
			if (slot == params_.pidf_slot_)
				return true;
			params_.pidf_slot_ = slot;

			// If dynamic reconfigure is running update
			// the reported config there with the new internal
			// state
			syncDynamicReconfigure();

			talon_->setPidfSlot(params_.pidf_slot_);
			return true;
		}

		virtual void setNeutralOutput(void)
		{
			talon_->setNeutralOutput();
		}

		virtual void setIntegralAccumulator(double iaccum)
		{
			talon_->setIntegralAccumulator(iaccum);
		}

		virtual void setOverrideSoftLimitsEnable(bool enable)
		{
			if (enable == params_.softlimits_override_enable_)
				return;
			params_.softlimits_override_enable_ = enable;
			syncDynamicReconfigure();
			talon_->setOverrideSoftLimitsEnable(enable);
		}

		virtual void setPeakCurrentLimit(int amps)
		{
			if (amps == params_.current_limit_peak_amps_)
				return;
			params_.current_limit_peak_amps_ = amps;

			syncDynamicReconfigure();

			talon_->setPeakCurrentLimit(params_.current_limit_peak_amps_);
		}

		virtual void setPeakCurrentDuration(int msec)
		{
			if (msec == params_.current_limit_peak_msec_)
				return;
			params_.current_limit_peak_msec_ = msec;

			syncDynamicReconfigure();

			talon_->setPeakCurrentDuration(params_.current_limit_peak_msec_);
		}

		virtual void setContinouousCurrentLimit(int amps)
		{
			if (amps == params_.current_limit_continuous_amps_)
				return;
			params_.current_limit_continuous_amps_ = amps;

			syncDynamicReconfigure();

			talon_->setContinuousCurrentLimit(params_.current_limit_continuous_amps_);
		}

		virtual void setCurrentLimitEnable(bool enable)
		{
			if (enable == params_.current_limit_enable_)
				return;
			params_.current_limit_enable_ = enable;

			syncDynamicReconfigure();

			talon_->setCurrentLimitEnable(params_.current_limit_enable_);
		}

		virtual void setForwardSoftLimitThreshold(double threshold)
		{
			if (threshold == params_.softlimit_forward_threshold_)
				return;
			params_.softlimit_forward_threshold_= threshold;

			syncDynamicReconfigure();
			talon_->setForwardSoftLimitThreshold(threshold);
		}

		virtual void setForwardSoftLimitEnable(bool enable)
		{
			if (enable == params_.softlimit_forward_enable_)
				return;
			params_.softlimit_forward_enable_= enable;

			syncDynamicReconfigure();
			talon_->setForwardSoftLimitEnable(enable);
		}

		virtual void setReverseSoftLimitThreshold(double threshold)
		{
			if (threshold == params_.softlimit_forward_threshold_)
				return;
			params_.softlimit_forward_threshold_= threshold;

			syncDynamicReconfigure();
			talon_->setReverseSoftLimitThreshold(threshold);
		}

		virtual void setReverseSoftLimitEnable(bool enable)
		{
			if (enable== params_.softlimit_forward_enable_)
				return;
			params_.softlimit_forward_enable_= enable;

			syncDynamicReconfigure();
			talon_->setReverseSoftLimitEnable(enable);
		}
		virtual void setSelectedSensorPosition(double position)
		{
			talon_->setSelectedSensorPosition(position);
		}

		virtual void clearStickyFaults(void)
		{
			talon_->setClearStickyFaults();
		}

		virtual void setMotionCruiseVelocity(double velocity)
		{
			if (velocity == params_.motion_cruise_velocity_)
				return;
			params_.motion_cruise_velocity_ = velocity;

			syncDynamicReconfigure();

			talon_->setMotionCruiseVelocity(params_.motion_cruise_velocity_);
		}

		virtual void setMotionAcceleration(double acceleration)
		{
			if (acceleration == params_.motion_acceleration_)
				return;
			params_.motion_acceleration_ = acceleration;

			syncDynamicReconfigure();

			talon_->setMotionAcceleration(params_.motion_acceleration_);
		}
		virtual double getMotionCruiseVelocity(void)
		{
			return params_.motion_cruise_velocity_;
		}

		virtual double getMotionAcceleration(void)
		{
			return params_.motion_acceleration_;
		}
		virtual void setMotionControlFramePeriod(int msec)
		{
			if (msec == params_.motion_control_frame_period_)
				return;
			params_.motion_control_frame_period_ = msec;

			syncDynamicReconfigure();

			talon_->setMotionControlFramePeriod(params_.motion_control_frame_period_);
		}

		virtual void clearMotionProfileTrajectories(void)
		{
			talon_->setClearMotionProfileTrajectories();
		}

		virtual void clearMotionProfileHasUnderrun(void)
		{
			talon_->setClearMotionProfileHasUnderrun();
		}

		virtual void pushMotionProfileTrajectory(const hardware_interface::TrajectoryPoint &traj_pt)
		{
			talon_->PushMotionProfileTrajectory(traj_pt);
		}

		double getPosition(void) const
		{
			return talon_.state()->getPosition();
		}

	protected:
		hardware_interface::TalonCommandHandle                          talon_;
		TalonCIParams                                                   params_;
		std::shared_ptr<dynamic_reconfigure::Server<TalonConfigConfig>> srv_;
		std::shared_ptr<boost::recursive_mutex>                         srv_mutex_;

#if 1
		std::vector<hardware_interface::TalonCommandHandle>                          follower_talons_;
		std::vector<std::shared_ptr<dynamic_reconfigure::Server<TalonConfigConfig>>> follower_srvs_;
		std::vector<std::shared_ptr<boost::recursive_mutex>>                         follower_srv_mutexes_;
#endif

		// Used to set initial (and only) talon
		// mode for FixedMode derived classes
		virtual bool setInitialMode(void)
		{
			ROS_INFO_STREAM("Talon " << talon_.getName() << " Base class setInitialMode");
			return true;
		}


	private :
		virtual bool init(hardware_interface::TalonCommandInterface *tci,
							ros::NodeHandle &n,
							hardware_interface::TalonCommandHandle &talon,
							std::shared_ptr<boost::recursive_mutex> srv_mutex,
							std::shared_ptr<dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig>> &srv,
							bool update_params,
							bool dynamic_reconfigure = false)
		{
			ROS_WARN("init start");
			// Read params from startup and intialize Talon using them
			TalonCIParams params;
			if (!readParams(n, params))
			   return false;
			ROS_WARN("init past readParams");

			talon = tci->getHandle(params.joint_name_);
			if (!writeParamsToHW(params, update_params))
				return false;

			ROS_WARN("init past writeParamsToHW");
			if (dynamic_reconfigure)
			{
				// Create dynamic_reconfigure Server. Pass in n
				// so that all the vars for the class are grouped
				// under the node's name.  Doing so allows multiple
				// copies of the class to be started, each getting
				// their own namespace.
				srv_mutex = std::make_shared<boost::recursive_mutex>();
				srv = std::make_shared<dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig>>(*srv_mutex_, n);

				ROS_WARN("init updateConfig");
				// Without this, the first call to callback()
				// will overwrite anything passed in from the
				// launch file
				srv->updateConfig(params_.toConfig());

				ROS_WARN("init setCallback");
				// Register a callback function which is run each
				// time parameters are changed using
				// rqt_reconfigure or the like
				srv->setCallback(boost::bind(&TalonControllerInterface::callback, this, _1, _2));
			}
			ROS_WARN("init returning");

			return true;
		}
		// If dynamic reconfigure is running then update
		// the reported config there with the new internal
		// state
		void syncDynamicReconfigure(void)
		{
			if (srv_)
			{
				TalonConfigConfig config(params_.toConfig());
				//boost::recursive_mutex::scoped_lock lock(*srv_mutex_);
				srv_->updateConfig(config);
			}
		}
};

// A derived class which disables mode switching. Any
// single-mode CI class should derive from this class
class TalonFixedModeControllerInterface : public TalonControllerInterface
{
	protected:
		// Disable changing mode for controllers derived
		// from this class
		void setMode(hardware_interface::TalonMode /*mode*/) override
		{
			ROS_WARN("Can't reset mode using this TalonControllerInterface");
		}
};
class TalonPercentOutputControllerInterface : public TalonFixedModeControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_PercentOutput);
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Percent Output mode");
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
};

class TalonFollowerControllerInterface : public TalonFixedModeControllerInterface
{
	public:
		bool initWithNode(hardware_interface::TalonCommandInterface *tci,
						  hardware_interface::TalonStateInterface   *tsi,
						  ros::NodeHandle &n,
						  bool dynamic_reconfigure = false) override
		{
			if (!tsi)
			{
				ROS_ERROR("NULL TalonStateInterface in TalonFollowerCommandInterface");
				return false;
			}

			// Call base-class init to load config params
			if (!TalonControllerInterface::initWithNode(tci, tsi, n, dynamic_reconfigure))
			{
				ROS_ERROR("TalonFollowerController base initWithNode failed");
				return false;
			}

			std::string follow_joint_name;
			if (!n.getParam("follow_joint", follow_joint_name))
			{
				ROS_ERROR("No follow joint specified for TalonFollowerControllerInterface");
				return false;
			}

			hardware_interface::TalonStateHandle follow_handle = tsi->getHandle(follow_joint_name);
			const int follow_can_id = follow_handle->getCANID();

			// Set the mode and CAN ID of talon to follow at init time -
			// since this class is derived from the FixedMode class
			// these can't be reset. Hopefully we never have a case
			// where a follower mode Talon changes which other
			// Talon it is following during a match?
			talon_->setMode(hardware_interface::TalonMode_Follower);
			talon_->set(follow_can_id);

			ROS_INFO_STREAM("Launching follower " << params_.joint_name_ << " to follow CAN ID " << follow_can_id << " (" << follow_handle.getName() << ")");
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
		void setCommand(const double /*command*/) override
		{
			ROS_WARN("Can't set a command in follower mode!");
		}
};

// Use this to create any methods common to all
// Close Loop modes, if any
class TalonCloseLoopControllerInterface : public TalonFixedModeControllerInterface
{

};

class TalonPositionCloseLoopControllerInterface : public TalonCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to position close loop mode
			talon_->setMode(hardware_interface::TalonMode_Position);

			//RG: We should consider setting the PID config for position to default to 1
			//Sorting PID values based on type (position vs velocity) seems like a good idea and
			//having a position and a velocity mode is relatively common for drive trains
			//In other cases it will make it clearer how the PID vals are used
			//KCJ - works for now, at least until we get virtual PID config slots going...
			//      at that point my hope is to have named PID configs rather than numbers
			//      and then add initial PID mode as one of the yaml params
			setPIDFSlot(1); // pick a default?
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Close Loop Position mode");

			return true;
		}
};

class TalonVelocityCloseLoopControllerInterface : public TalonCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			talon_->setMode(hardware_interface::TalonMode_Velocity);
			setPIDFSlot(0); // pick a default?
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Close Loop Velocity mode");

			return true;
		}
};

class TalonCurrentControllerCloseLoopInterface : public TalonCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to current close loop mode
			talon_->setMode(hardware_interface::TalonMode_Current);
			setPIDFSlot(0); // pick a default?
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Close Loop Current mode");
			return true;
		}
};

class TalonMotionProfileControllerInterface : public TalonCloseLoopControllerInterface // double check that this works
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_MotionProfile);
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in MotionProfile mode");
			return true;
		}
};

//RG: I can think of few to no situations were we would have a talon in motion magic mode for an entire match
//Honestly I wouldn't ever use motion magic mode, I would use the MotionProfile mode (above)
// KCJ -- in general the code we actually use will get a lot more attention. Not sure if that
// means we should pull out less-tested stuff like this or leave it in and fix it if
// we need it at some point?
class TalonMotionMagicCloseLoopControllerInterface : public TalonCloseLoopControllerInterface // double check that this works
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_MotionMagic);
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in MotionMagic mode");
			return true;
		}
};

} // namespace
