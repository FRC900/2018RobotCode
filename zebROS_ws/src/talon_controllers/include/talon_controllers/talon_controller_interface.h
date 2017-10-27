
#pragma once
#include <talon_interface/talon_interface.h>

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
		TalonCIParams(void)
		{
			for (int slot = 0; slot < 2; slot++)
			{
				p_[slot] =  0.0;
				i_[slot] =  0.0;
				d_[slot] =  0.0;
				f_[slot] =  0.0;
				i_zone_[slot] = 0.0;
			}
		}

		bool readJointName(ros::NodeHandle &n)
		{
			if (!n.getParam("joint", joint_name_))
			{
				ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
				return false;
			}
			return true;
		}
		double findPidParam(std::string param_type, XmlRpc::XmlRpcValue &pid_params)
		{
			if (!pid_params.hasMember(param_type))
				throw std::runtime_error(param_type+" was not specified");
			XmlRpc::XmlRpcValue& param = pid_params[param_type];
 			if (!param.valid())
				throw std::runtime_error(param_type+" was not a valid type");
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
		bool readCloseLoopParams(ros::NodeHandle &n)
		{
			// Figure out how these will be stored
			// Look at loop in frc_robot_interface.cpp 
			// FRCRobotInterface constructor for example
			// of loading from an array of maps
			// Change names to match those in the example
			// yaml file, and assign values to p_[], i_[], etc
			// arrays.
		XmlRpc::XmlRpcValue pid_param_list;
		if (!n.getParam("close_loop_values", pid_param_list))
			throw std::runtime_error("No joints were specified.");
		for (int i = 0; i < pid_param_list.size(); i++)
		{
			XmlRpc::XmlRpcValue &pid_params = pid_param_list[i];

			p_[i]=findPidParam("p",pid_params);
			i_[i]=findPidParam("i",pid_params);
			d_[i]=findPidParam("d",pid_params);
			f_[i]=findPidParam("f",pid_params);
			i_zone_[i]=findPidParam("i_zone",pid_params);
			std::cout << "p_value = " << p_[i] << " i_value = " << i_[i] << " d_value = " << d_[i] << " f_value = " << f_[i] 			<< "i _zone value = " << i_zone_[i]<< std::endl;
			// Eventually add a list of valid modes for this joint?
		}
			return true;
		}
		std::string joint_name_;
		double p_[2];
		double i_[2];
		double d_[2];
		double f_[2];
		double i_zone_[2];
};

class TalonControllerInterface
{
	public:
		// Standardize format for reading params for 
		// motor controller
		virtual bool readParams(ros::NodeHandle &n, TalonCIParams &params)
		{
			return params.readJointName(n);
		}
		virtual bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
									const TalonCIParams &params)
		{
			talon_ = hw->getHandle(params.joint_name_);
			talon_->set(0);
			// perform any other generic initializtion here
			return true;
		}
		// Useful for cases where there's no need
		// to modify the parameters after reading
		// them
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *hw,
						  ros::NodeHandle &n)
		{
			TalonCIParams params;
			return readParams(n, params) && initWithParams(hw, params);
		}
		virtual void setCommand(const double command)
		{
			talon_->set(command);
		}
	protected:
		hardware_interface::TalonCommandHandle talon_;
};

class TalonPercentVbusControllerInterface : public TalonControllerInterface
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

// Maybe make a TalonPIDControllerInterface instead, and then
// derive a PositionPID and VeloctyPID from that?
class TalonCloseLoopControllerInterface : public TalonControllerInterface
{
	public:
		bool readParams(ros::NodeHandle &n, TalonCIParams &params) override
		{
			return TalonControllerInterface::readParams(n, params) &&
				params.readCloseLoopParams(n);
		}
		bool initWithParams(hardware_interface::TalonCommandInterface* hw, 
				  const TalonCIParams &params) override
		{
			// Call base class init for common setup code
			if (!TalonControllerInterface::initWithParams(hw, params))
				return false;
			// perform any PositionPID specific 
			// initializtion here
			//
			// for (int slot = 0; slot < 2; slot++)
			// {
			// talon_->setPgain(slot, p[slot]); 
			// // and I, D, F, Izone
			// }
			return true;
		}
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
			talon_->setMode(hardware_interface::TalonMode_PositionCloseLoop);

			// Call close loop init to set PIDF & IZone
			if (!TalonCloseLoopControllerInterface::initWithParams(hw, params))
				return false;
			return true;
		}
};

} // namespace
