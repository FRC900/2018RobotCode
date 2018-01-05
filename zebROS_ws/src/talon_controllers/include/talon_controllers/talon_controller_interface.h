#pragma once
#include <dynamic_reconfigure/server.h>
#include <talon_interface/talon_command_interface.h>
#include <talon_controllers/TalonConfigConfig.h>

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
				follow_can_id_ (-1),
				p_ {0, 0},
				i_ {0, 0},
				d_ {0, 0},
				f_ {0, 0},
				izone_ {0, 0},
				allowable_closed_loop_error_{0, 0}, // need better defaults
				max_integral_accumulator_{0, 0},
				pidf_slot_(0),
				invert_output_ (false),
				sensor_phase_(false),
				neutral_mode_(hardware_interface::NeutralMode_Uninitialized),
			    feedback_type_(hardware_interface::FeedbackDevice_Uninitialized),
				ticks_per_rotation_(4096),
				closed_loop_ramp_(0.),
				open_loop_ramp_(0.),
				peak_output_forward_(100.),
				peak_output_reverse_(100.),
				nominal_output_forward_(100.),
				nominal_output_reverse_(100.),
				neutral_deadband_(0.),
				voltage_compensation_saturation_(0),
				voltage_measurement_filter_(0),
				voltage_compensation_enable_(false),
				softlimit_forward_threshold_(0.0),
				softlimit_forward_enable_(false),
				softlimit_reverse_threshold_(0.0),
				softlimit_reverse_enable_(false),
				softlimits_override_enable_(false),
				current_limit_peak_amps_(0),
				current_limit_peak_msec_(0),
				current_limit_continuous_amps_(0),
				current_limit_enable_(false),
				motion_control_frame_period_(20) // Guess at 50Hz default?
		{
		}

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

			softlimit_forward_threshold_ = config.softlimit_forward_threshold;
			softlimit_forward_enable_ = config.softlimit_forward_enable;
			softlimit_reverse_threshold_ = config.softlimit_reverse_threshold;
			softlimit_reverse_enable_ = config.softlimit_reverse_enable;
			softlimits_override_enable_ = config.softlimits_override_enable;

			current_limit_peak_amps_ = config.current_limit_peak_amps;
			current_limit_peak_msec_ = config.current_limit_peak_msec;
			current_limit_continuous_amps_ = config.current_limit_continuous_amps;
			current_limit_enable_ = config.current_limit_enable;
			motion_control_frame_period_ = config.motion_control_frame_period;
		}

		// Copy from internal state to TalonConfigConfig state
		TalonConfigConfig getConfig(void) const
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
			config.softlimit_forward_threshold = softlimit_forward_threshold_;
			config.softlimit_forward_enable = softlimit_forward_enable_;
			config.softlimit_reverse_threshold = softlimit_reverse_threshold_;
			config.softlimit_reverse_enable = softlimit_reverse_enable_;
			config.softlimits_override_enable = softlimits_override_enable_;
			config.current_limit_peak_amps = current_limit_peak_amps_;
			config.current_limit_peak_msec = current_limit_peak_msec_;
			config.current_limit_continuous_amps = current_limit_continuous_amps_;
			config.current_limit_enable = current_limit_enable_;
			config.motion_control_frame_period = motion_control_frame_period_;
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
			else if (feedback_type_name == "Inertial")
				feedback_type_ = hardware_interface::FeedbackDevice_Inertial;
			else if (feedback_type_name == "RemoteSensor")
				feedback_type_ = hardware_interface::FeedbackDevice_RemoteSensor;
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
			return true;
		}

		// Read the name of a joint (talon) to follow.  Figure
		// out which CAN ID that Talon is configured as here
		bool readFollowerID(ros::NodeHandle &n, hardware_interface::TalonStateInterface *tsi)
		{
			std::string follow_joint_name;
			if (tsi && n.getParam("follow_joint", follow_joint_name))
			{
				hardware_interface::TalonStateHandle follow_handle = tsi->getHandle(follow_joint_name);
				follow_can_id_ = follow_handle->getCANID();
			}
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
			if(pid_param_list.size() <= 2)
			{
				for (int i = 0; i < pid_param_list.size(); i++)
				{
					XmlRpc::XmlRpcValue &pidparams_ = pid_param_list[i];

					p_[i]=findFloatParam("p",pidparams_);
					i_[i]=findFloatParam("i",pidparams_);
					d_[i]=findFloatParam("d",pidparams_);
					f_[i]=findFloatParam("f",pidparams_);
					izone_[i]=findIntParam("i_zone",pidparams_);
					allowable_closed_loop_error_[i]=findIntParam("allowable_closed_loop_error", pidparams_);
					max_integral_accumulator_[i]=findFloatParam("max_integral_accumulator",pidparams_);
					std::cout << "p_value = " << p_[i] << " i_value = " << i_[i] << " d_value = " << d_[i] << " f_value = " << f_[i] << " i _zone value = " << izone_[i]<< std::endl;
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
			double double_val;
			if (n.getParam("closed_loop_ramp", double_val))
				closed_loop_ramp_ = double_val;
			if (n.getParam("open_loop_ramp", double_val))
				open_loop_ramp_ = double_val;
			if (n.getParam("peak_output_forward", double_val))
				peak_output_forward_ = double_val;
			if (n.getParam("peak_output_reverse", double_val))
				peak_output_reverse_ = double_val;
			if (n.getParam("nominal_output_forward", double_val))
				nominal_output_forward_ = double_val;
			if (n.getParam("nominal_output_reverse", double_val))
				nominal_output_reverse_ = double_val;
			if (n.getParam("neutral_deadband", double_val))
				neutral_deadband_ = double_val;
			return true;
		}
		bool readVoltageCompensation(ros::NodeHandle &n)
		{
			int params_read = 0;
			double double_val;
			if (n.getParam("voltage_compensation_saturation", double_val))
			{
				voltage_compensation_saturation_ = double_val;
				params_read += 1;
			}
			int int_val;
			if (n.getParam("voltage_measurement_filter", int_val))
			{
				voltage_measurement_filter_ = int_val;
				params_read += 1;
			}
			bool bool_val;
			if (n.getParam("voltage_compensation_enable", bool_val))
			{
				voltage_compensation_enable_ = bool_val;
				if (bool_val && (params_read < 2))
					ROS_WARN("Not all voltage compensation params set before enabling - using defaults of 0 might not work as expected");
			}
			return true;
		}

		bool readSoftLimits(ros::NodeHandle &n)
		{
			double double_val;
			bool bool_val;
			int param_count = 0;
			if (n.getParam("softlimit_forward_threshold", double_val))
			{
				softlimit_forward_threshold_ = double_val;
				param_count = 1;
			}
			if (n.getParam("softlimit_forward_enable", bool_val))
			{
				softlimit_forward_enable_ = bool_val;
				if (bool_val && (param_count == 0))
					ROS_WARN("Enabling forward softlimits without setting threshold");
			}
			param_count = 0;
			if (n.getParam("softlimit_reverse_threshold", double_val))
			{
				softlimit_reverse_threshold_ = double_val;
				param_count = 1;
			}
			if (n.getParam("softlimit_reverse_enable", bool_val))
			{
				softlimit_reverse_enable_ = bool_val;
				if (bool_val && (param_count == 0))
					ROS_WARN("Enabling forward softlimits without setting threshold");
			}
			if (n.getParam("softlimits_override_enable", bool_val))
				softlimits_override_enable_ = bool_val;
			return true;
		}

		bool readCurrentLimits(ros::NodeHandle &n)
		{
			int params_read = 0;
			int int_val;
			if (n.getParam("current_limit_peak_amps", int_val))
			{
				current_limit_peak_amps_ = int_val;
				params_read += 1;
			}
			if (n.getParam("current_limit_peak_msec", int_val))
			{
				current_limit_peak_msec_ = int_val;
				params_read += 1;
			}
			if (n.getParam("current_limit_continusous_amps", int_val))
			{
				current_limit_continuous_amps_ = int_val;
				params_read += 1;
			}
			bool bool_val;
			if (n.getParam("current_limit_enable", bool_val))
			{
				current_limit_enable_ = bool_val;
				if (bool_val && (params_read < 3))
					ROS_WARN("Not all current limits set before enabling - using defaults of 0 might not work as expected");
			}
			return true;
		}

		bool readMotionControl(ros::NodeHandle &n)
		{
			n.getParam("motion_control_frame_period", motion_control_frame_period_);
			return true;
		}

		// TODO : Keep adding config items here
		std::string joint_name_;
		int    follow_can_id_;
		double  p_[2];
		double  i_[2];
		double  d_[2];
		double  f_[2];
		int    izone_[2];
		int    allowable_closed_loop_error_[2];
		double  max_integral_accumulator_[2];
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
		double softlimit_forward_threshold_;
		bool   softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool   softlimit_reverse_enable_;
		bool   softlimits_override_enable_;
		int    current_limit_peak_amps_;
		int    current_limit_peak_msec_;
		int    current_limit_continuous_amps_;
		bool   current_limit_enable_;
		int    motion_control_frame_period_;

	private:
		// Read a double named <param_type> from the array/map
		// in params
		double findFloatParam(std::string param_type, XmlRpc::XmlRpcValue &params) const
		{
			if (!params.hasMember(param_type))
				return 0;
			XmlRpc::XmlRpcValue& param = params[param_type];
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
			XmlRpc::XmlRpcValue& param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a valid int type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
				return (int)param;
			else
				throw std::runtime_error("A non-int value was passed for" + param_type);
			return 0;
		}

		// Read a bool named <param_type> from the array/map
		// in params
		bool findBoolParam(std::string param_type, XmlRpc::XmlRpcValue &params) const
		{
			if (!params.hasMember(param_type))
				return false;
			XmlRpc::XmlRpcValue& param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a bool valid type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
				return (bool)param;
			else
				throw std::runtime_error("A non-bool value was passed for" + param_type);
			return false;
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
			srv_(nullptr)
		{
			srv_mutex_ = std::make_shared<boost::recursive_mutex>();
		}
		// Standardize format for reading params for 
		// motor controller
		virtual bool readParams(ros::NodeHandle &n, hardware_interface::TalonStateInterface *tsi)
		{
			return params_.readJointName(n) && 
				   params_.readFollowerID(n, tsi) && 
				   params_.readCloseLoopParams(n) &&
				   params_.readNeutralMode(n) &&
				   params_.readInverts(n) &&
				   params_.readFeedbackType(n) &&
				   params_.readOutputShaping(n) &&
				   params_.readVoltageCompensation(n) &&
				   params_.readSoftLimits(n) &&
				   params_.readCurrentLimits(n) &&
				   params_.readMotionControl(n);
		}

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

		// Use data in params_ to actually set up Talon
		// hardware. Make this a separate method outside of
		// init() so that dynamic reconfigure callback can write
		// values using this method at any time
		virtual bool writeParamsToHW(const TalonCIParams &params)
		{
			// Save copy of params written to HW
			// so they can be queried later?
			params_ = params;

			// perform additional hardware init here
			// but don't set mode - either force the caller to
			// set it or use one of the derived, fixed-mode
			// classes instead
			for (int i = 0; i < 2; i++) {
				talon_->setP(params_.p_[i], i);
				talon_->setI(params_.i_[i], i);
				talon_->setD(params_.d_[i], i);
				talon_->setF(params_.f_[i], i);
				talon_->setIZ(params_.izone_[i], i);
				// TODO : I'm worried about these. We need
				// better default values than 0.0
				talon_->setAllowableClosedloopError(params_.allowable_closed_loop_error_[i], i);
				talon_->setMaxIntegralAccumulator(params_.max_integral_accumulator_[i], i);
			}
			talon_->setPidfSlot(params_.pidf_slot_);
			talon_->setNeutralMode(params_.neutral_mode_);

			talon_->setInvert(params_.invert_output_);
			talon_->setSensorPhase(params_.sensor_phase_);

			talon_->setClosedloopRamp(params_.closed_loop_ramp_);
			talon_->setOpenloopRamp(params_.open_loop_ramp_);
			talon_->setPeakOutputForward(params_.peak_output_forward_);
			talon_->setPeakOutputReverse(params_.peak_output_reverse_);
			talon_->setNominalOutputForward(params_.nominal_output_forward_);
			talon_->setNominalOutputReverse(params_.nominal_output_reverse_);
			talon_->setNeutralDeadband(params_.neutral_deadband_);

			talon_->setVoltageCompensationSaturation(params_.voltage_compensation_saturation_);
			talon_->setVoltageMeasurementFilter(params_.voltage_measurement_filter_);
			talon_->setVoltageCompensationEnable(params_.voltage_compensation_enable_);

			talon_->setForwardSoftLimitThreshold(params.softlimit_forward_threshold_);
			talon_->setForwardSoftLimitEnable(params.softlimit_forward_enable_);
			talon_->setReverseSoftLimitThreshold(params.softlimit_reverse_threshold_);
			talon_->setReverseSoftLimitEnable(params.softlimit_reverse_enable_);
			talon_->setOverrideSoftLimitsEnable(params.softlimits_override_enable_);

			talon_->setPeakCurrentLimit(params_.current_limit_peak_amps_);
			talon_->setPeakCurrentDuration(params_.current_limit_peak_msec_);
			talon_->setContinuousCurrentLimit(params_.current_limit_continuous_amps_);
			talon_->setCurrentLimitEnable(params_.current_limit_enable_);

			talon_->setMotionControlFramePeriod(params_.motion_control_frame_period_);
			return true;
		}

		void callback(talon_controllers::TalonConfigConfig &config, uint32_t level)
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

			TalonCIParams params(config);
			
			writeParamsToHW(params);
		}

		// Read params from config file and use them to 
		// initialize the Talon hardware
		// Useful for the hopefully common case where there's 
		// no need to modify the parameters after reading
		// them
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
							 	  hardware_interface::TalonStateInterface *tsi,
								  ros::NodeHandle &n)
		{
			// Read params from startup and intialize
			// Talon using them
			bool result = readParams(n, tsi) && initWithParams(tci, params_);

			if (result)
			{
				// Create dynamic_reconfigure Server. Pass in n
				// so that all the vars for the class are grouped
				// under the node's name.  Doing so allows multiple
				// copies of the class to be started, each getting
				// their own namespace.
				srv_ = std::make_shared<dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig>>(*srv_mutex_, n);

				// Without this, the first call to callback() 
				// will overwrite anything passed in from the
				// launch file
				srv_->updateConfig(params_.getConfig());

				// Register a callback function which is run each
				// time parameters are changed using 
				// rqt_reconfigure or the like
				srv_->setCallback(boost::bind(&TalonControllerInterface::callback, this, _1, _2));
			}

			return result;
		}

		// Set the setpoint for the motor controller
		virtual void setCommand(const double command)
		{
			talon_->set(command);
		}

		// Set the mode of the motor controller
		virtual void setMode(const hardware_interface::TalonMode mode)
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

		virtual void setMotionControlFramePeriod(int msec)
		{
			if (msec == params_.motion_control_frame_period_)
				return;
			params_.motion_control_frame_period_= msec;

			syncDynamicReconfigure();

			talon_->setMotionControlFramePeriod(params_.motion_control_frame_period_);
		}

		virtual void ClearMotionProfileTrajectories(void)
		{
			talon_->setClearMotionProfileTrajectories();
		}

		virtual void ClearMotionProfileHasUnderrun(void)
		{
			talon_->setClearMotionProfileHasUnderrun();
		}

		virtual void PushMotionProfileTrajectory(const hardware_interface::TrajectoryPoint &traj_pt)
		{
			talon_->PushMotionProfileTrajectory(traj_pt);
		}

		virtual void ProcessMotionProfileBuffer(void)
		{
			talon_->setProcessMotionProfileBuffer();
		}

		double getPosition(void) const
		{
			return talon_.state()->getPosition();
		}

	protected:
		hardware_interface::TalonCommandHandle talon_;
		TalonCIParams                          params_;
		std::shared_ptr<dynamic_reconfigure::Server<TalonConfigConfig>> srv_;
		std::shared_ptr<boost::recursive_mutex> srv_mutex_;

	private :
		// If dynamic reconfigure is running update
		// the reported config there with the new internal
		// state
		void syncDynamicReconfigure(void)
		{
			if (srv_)
			{
				TalonConfigConfig config(params_.getConfig());
				boost::recursive_mutex::scoped_lock lock(*srv_mutex_);
				srv_->updateConfig(config);
			}
		}
};

// A derived class which disables mode switching. Any
// single-mode CI class should derive from this class
class TalonFixedModeControllerInterface : public TalonControllerInterface
{
	public:
		// Disable changing mode for controllers derived
		// from this class
		void setMode(const hardware_interface::TalonMode /*mode*/) override
		{
			ROS_WARN("Can't reset mode using this TalonControllerInterface");
		}
};
class TalonPercentOutputControllerInterface : public TalonFixedModeControllerInterface
{
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
				  const TalonCIParams &params) override
		{
			// Call base-class init to load config params
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_PercentOutput);
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
};

class TalonFollowerControllerInterface : public TalonFixedModeControllerInterface
{
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
				  const TalonCIParams &params) override
		{
			// Call base-class init to load config params
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;
			if (params.follow_can_id_ < 0 || params.follow_can_id_ > 99)
				throw std::runtime_error("Invalid follower CAN ID");

			// Set the mode and CAN ID of talon to follow at init time - 
			// since this class is derived from the FixedMode class
			// these can't be reset. Hopefully we never have a case
			// where a follower mode Talon changes which other
			// Talon it is following during a match?
			talon_->setMode(hardware_interface::TalonMode_Follower);
			talon_->set(params.follow_can_id_);

			std::cout << "Launching follower Talon SRX " << params.joint_name_ << " to follow CAN ID " << params.follow_can_id_ << std::endl;
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
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
						    const TalonCIParams &params) override
		{
			// Call base class init for common setup code
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;

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

			return true;
		}
};

class TalonVelocityCloseLoopControllerInterface : public TalonCloseLoopControllerInterface
{
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
						    const TalonCIParams &params) override
		{
			// Call base class init for common setup code
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;

			// Set to speed close loop mode
			talon_->setMode(hardware_interface::TalonMode_Velocity);
			setPIDFSlot(0); // pick a default?

			return true;
		}
};

class TalonCurrentControllerCloseLoopInterface : public TalonCloseLoopControllerInterface
{
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
						    const TalonCIParams &params) override
		{
			// Call base class init for common setup code
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;

			// Set to current close loop mode
			talon_->setMode(hardware_interface::TalonMode_Current);
			setPIDFSlot(0); // pick a default?

			return true;
		}
};

class TalonMotionProfileControllerInterface : public TalonCloseLoopControllerInterface // double check that this works
{
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
				  const TalonCIParams &params) override
		{
			// Call base-class init to load config params
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_MotionProfile);
			return true;
		}
};

//RG: I can think of few to no situations were we would have a talon in motion magic mode for an entire match
//Honestly I wouldn't ever use motion magic mode, I would use the MotionProfile mode (above)
// KCJ -- in general the code we actually use will get a lot more attention. Not sure if that
// means we should pull out less-tested stuff like this or leave it in and fix it if
// we need it at some point?
class TalonMotionMagicControllerInterface : public TalonCloseLoopControllerInterface // double check that this works
{
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
							const TalonCIParams &params) override
		{
			// Call base-class init to load config params
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_MotionMagic);
			return true;
		}
};

} // namespace
