#ifndef MECH_CONTROLLER
#define MECH_CONTROLLER

#include <ros/ros.h>
#include <vector>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h>
#include <talon_interface/talon_state_interface.h>
#include <talon_controllers/talon_controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <mech_controller/SetTwoMotors.h>
#include <mech_controller/TwoMotor.h>
#include <atomic>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h>

namespace mech_controller
{ 
//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class MechController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface>
{
	public:
		MechController()
		{
		}

		//should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
		//answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
		virtual bool init(hardware_interface::RobotHW *hw,
							ros::NodeHandle						&root_nh,
							ros::NodeHandle						&controller_nh);
		virtual void starting(const ros::Time &time);
		virtual void update(const ros::Time & time, const ros::Duration& period);
		virtual void stopping(const ros::Time &time);
	
		virtual bool cmdService(mech_controller::SetPosition::Request &req, mech_controller::SetPosition::Response &/*res*/);
		virtual void mechPosCallback(const std_msgs::Float64 &command);

	private:
		talon_controllers::TalonPercentOutputControllerInterface joint_1; //interface for the actual joint
		talon_controllers::TalonPercentOutputControllerInterface joint_2; //interface for the actual joint
		realtime_tools::RealtimeBuffer<std::vector<float>> command_; //this is the buffer for vel commands to be published. 
		ros::Subscriber sub_command_;
		ros::ServiceServer service_command_;
}; //class

} //namespace
#endif
