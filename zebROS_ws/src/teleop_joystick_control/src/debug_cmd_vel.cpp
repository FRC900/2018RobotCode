#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <ros/console.h>
#include <teleop_joystick_control/teleopJoystickCommands.h>
#include <ros_control_boilerplate/JoystickState.h>

void JSHeaderCallback(const std_msgs::Header &msg)
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

void JSHeaderCallback2(const std_msgs::Header &msg)
{
	//teleop_delay = msg.stamp.sec;
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


void HWHeaderCallback(const ros_control_boilerplate::JoystickState &msg)
{
	//hw_delay = msg.header.stamp.sec;
	static double average_delay = 0;
	static int count = 0;

	count++;
	if (count == 100)
	{
		ROS_INFO_STREAM("delay = " << average_delay << ". location = hw");
		count = 0;
		average_delay = 0;
	}
	else 
	{ 
		average_delay += (ros::Time::now().toSec() - msg.header.stamp.sec) / 100.0;
	}
}

void HWHeaderCallback(const ros_control_boilerplate::JoystickState &msg)
{
	static double average_delay = 0;
	static int count = 0;

	count++;
	if (count == 100)
	{
		ROS_INFO_STREAM("delay = " << average_delay << " . location hw");
		count = 0;
		average_delay = 0;
	}
	else 
	{ 
		average_delay += (ros::Time::now().toSec() - msg.header.stamp.sec) / 100.0;
	}
}
	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "debug_cmd_vel");
	ros::NodeHandle n;

	ros::Subscriber JS_subscriber = n.subscribe("test_header", 3, &JSHeaderCallback); //subscribing to teleopJoystickCommands loop
	ros::Subscriber JS_subscriber2 = n.subscribe("test_header", 3, &JSHeaderCallback2); //subscribing to teleopJoystickCommands loop
	ros::Subscriber hw_subscriber = n.subscribe("joystick_states", 1, &HWHeaderCallback); //subscribing to frcrobot_hw_interface loop

	ros::spin();
	return 0;
}
