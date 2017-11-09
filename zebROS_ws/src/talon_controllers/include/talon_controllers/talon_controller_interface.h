
#pragma once
#include <talon_interface/talon_command_interface.h>

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
		TalonCIParams(void)
		{
			for (int slot = 0; slot < 2; slot++)
			{
				p_[slot] = 0.0;
				i_[slot] = 0.0;
				d_[slot] = 0.0;
				f_[slot] = 0.0;
				i_zone_[slot] = 0.0;
			}
		}

		// Read a joint name from the given nodehandle's params
		bool readJointName(ros::NodeHandle &n, const std::string &param_name)
		{
			if (!n.getParam(param_name, joint_name_))
			{
				ROS_ERROR("No joint given (namespace: %s, param name: %s)", 
						  n.getNamespace().c_str(), param_name.c_str());
				return false;
			}
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

					p_[i]=findDoubleParam("p",pidparams_);
					i_[i]=findDoubleParam("i",pidparams_);
					d_[i]=findDoubleParam("d",pidparams_);
					f_[i]=findDoubleParam("f",pidparams_);
					i_zone_[i]=findDoubleParam("i_zone",pidparams_);
					std::cout << "p_value = " << p_[i] << " i_value = " << i_[i] << " d_value = " << d_[i] << " f_value = " << f_[i] << " i _zone value = " << i_zone_[i]<< std::endl;
				}
				return true;
			}
			else
			{
				throw std::runtime_error("More than two pid_param values");
			}
		}
		// TODO : Keep adding config items here
		std::string joint_name_;
		double p_[2];
		double i_[2];
		double d_[2];
		double f_[2];
		double i_zone_[2];
	private:
		// Read a double named <param_type> from the array/map
		// in pidparams
		double findDoubleParam(std::string param_type, XmlRpc::XmlRpcValue &pidparams) const
		{
			if (!pidparams.hasMember(param_type))
				return 0;
			XmlRpc::XmlRpcValue& param = pidparams[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a valid type");
			double ret;
			if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
				ret = param;
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
			{
				int temp = param;
				ret = temp;
			}
			else
				throw std::runtime_error("A non-double value was passed for" + param_type);
			return ret;
		}
};

class TalonControllerInterface
{
	public:
		// Standardize format for reading params for 
		// motor controller
		virtual bool readParams(ros::NodeHandle &n)
		{
			return params_.readJointName(n, "joint") && 
				   params_.readCloseLoopParams(n);
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
		virtual bool initWithParams(hardware_interface::TalonCommandInterface *hw, 
									const TalonCIParams &params)
		{
			talon_ = hw->getHandle(params.joint_name_);
			talon_->set(0); // make sure motors don't run until everything is configured
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
			return true;
		}

		// Read params from config file and use them to 
		// initialize the Talon hardware
		// Useful for the hopefully common case where there's 
		// no need to modify the parameters after reading
		// them
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *hw,
								  ros::NodeHandle &n)
		{
			return readParams(n) && initWithParams(hw, params_);
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
		virtual bool setPIDConfig(int config)
		{
			if ((config != 0) && (config != 1))
				return false;
			// Fill me in
			return true;
		}
	protected:
		hardware_interface::TalonCommandHandle talon_;
		TalonCIParams                          params_;
};

class TalonFixedModeControllerInterface : public TalonControllerInterface
{
	public:
		// Disable changing mode
		void setMode(const hardware_interface::TalonMode mode) override
		{
			ROS_WARN("Can't reset mode using this TalonControllerInterface");
		}
};

class TalonPercentVbusControllerInterface : public TalonFixedModeControllerInterface
{
	public:
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
				  const TalonCIParams &params) override
		{
			std::cout << "TalonPErcentVbusControllerInterface()" << std::endl;
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;
			talon_->setMode(hardware_interface::TalonMode_PercentVbus);
			return true;
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
			setPIDConfig(0); // pick a default?

			return true;
		}
};

} // namespace
