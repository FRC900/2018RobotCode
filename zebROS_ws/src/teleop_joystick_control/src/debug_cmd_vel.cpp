#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <ros/console.h>
#include <teleop_joystick_control/teleopJoystickCommands.h>
//#include <ros_control_boilerplate/frcrobot_hw_interface.h>
#include <ros_control_boilerplate/JoystickState.h>

void headerCallback(const std_msgs::Header &msg)
{
	static double average_delay = 0;
	static int count = 0;

	count++;
	if (count == 100)
	{
		ROS_INFO_STREAM("delay = " << average_delay << ". seq = " << msg.seq);
		count = 0;
		average_delay = 0;
	}
	else 
	{ 
		average_delay += (ros::Time::now().toSec() - msg.stamp.sec) / 100.0;
	}
}
/*
void otherHeaderCallback(const ros_control_boilerplate::JoystickStates &msg)
=======

void HWHeaderCallback(const ros_control_boilerplate::JoystickState &msg)
>>>>>>> c18bd38d8dcd1c48fca3ded1995c6a8074a4533a
{
	static double other_average_delay = 0;
	static int other_count = 0;

	other_count++;
	if (other_count == 100)
	{
		ROS_INFO_STREAM("delay = " << other_average_delay);
		other_count = 0;
		other_average_delay = 0;
	}
	else 
	{ 
		other_average_delay += (ros::Time::now().toSec() - msg.header.stamp.sec) / 100.0;
	}
}
	
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "debug_cmd_vel");
	ros::NodeHandle n;

	ros::Subscriber cmd_vel_subscriber1 = n.subscribe("test_header", 3, &headerCallback); //subscribing to teleopJoystickCommands loop
	//ros::Subscriber cmd_vel_subscriber2 = n.subscribe("joystick_states", 1, &HWHeaderCallback); //subscribing to frcrobot_hw_interface loop

	ros::spin();
	return 0;
}
