#ifndef PDP_STATE_CONTROLLER
#define PDP_STATE_CONTROLLER

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <iostream>
#include <realtime_tools/realtime_publisher.h>
#include <pdp_state_controller/PDPData.h>
#include <boost/shared_ptr.hpp>
#include <pdp_state_controller/pdp_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>

namespace pdp_state_controller
{
class PDPStateController: public controller_interface::Controller<hardware_interface::PDPStateInterface>
{
	public:
		PDPStateController() : publish_rate_(1.0) {}

		virtual bool init(hardware_interface::PDPStateInterface *hw,
							ros::NodeHandle 					&root_nh,
							ros::NodeHandle						&controller_nh);
		virtual void starting(const ros::Time &time);
		virtual void update(const ros::Time &time, const ros::Duration & );
		virtual void stopping(const ros::Time &time);

	private:
		hardware_interface::PDPStateHandle pdp_state_;
		boost::shared_ptr<realtime_tools::RealtimePublisher<pdp_state_controller::PDPData> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;

}; //class

} //namespace

#endif
