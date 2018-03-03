#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <ros/console.h>
#include <teleop_joystick_control/teleopJoystickCommands.h>

void headerCallback(const std_msgs::Header &msg)
{
	static int average_delay = 0;
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
		average_delay += (ros::Time::now().toSec() - msg.stamp.sec) / 100;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "debug_cmd_vel");
	ros::NodeHandle n;

	ros::Subscriber cmd_vel_subscriber = n.subscribe("test_header", 4, &headerCallback);

	ros::spin();
	return 0;
}
