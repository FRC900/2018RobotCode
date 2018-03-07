#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <ros/console.h>
#include <teleop_joystick_control/teleopJoystickCommands.h>
#include <ros_control_boilerplate/JoystickState.h>

static int teleop_delay;
static double teleop_averages;
static int hw_delay;
static double hw_averages;

//publish delays = [.5, .4, .8, .2]
void teleopHeaderCallback(const std_msgs::Header &msg)
{
	//teleop_delay = msg.stamp.sec;
	static double average_delay = 0;
	static int count = 0;
	count++;

		if (count == 100)
		{
			ROS_INFO_STREAM("delay = " << average_delay << ". location = " << msg.seq);
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
		ROS_INFO_STREAM("delay = " << average_delay);
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

	ros::Subscriber header_subscriber = n.subscribe("test_header", 3, &teleopHeaderCallback);
	ros::Subscriber header_subscriber2 = n.subscribe("joystick_states", 3, &HWHeaderCallback);
	
	/*
	static int count = 0;
	count++;

	if (count == 100)
	{
		ROS_INFO_STREAM("delay = " << teleop_delay << ". location = joystick");
		ROS_INFO_STREAM("delay = " << hw_delay << ". location = hw");
		count = 0;
		teleop_delay = 0;
		hw_delay = 0;
	}
	else 
	{ 
		teleop_averages += (ros::Time::now().toSec() - teleop_delay) / 100.0;
		hw_averages += (ros::Time::now().toSec() - hw_delay) / 100.0;
	}
=======
	ros::Subscriber cmd_vel_subscriber1 = n.subscribe("test_header", 3, &headerCallback); //subscribing to teleopJoystickCommands loop
	//ros::Subscriber cmd_vel_subscriber2 = n.subscribe("joystick_states", 1, &HWHeaderCallback); //subscribing to frcrobot_hw_interface loop
	*/

	ros::spin();
	return 0;
}
