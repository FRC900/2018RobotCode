#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <ros/console.h>
#include <teleop_joystick_control/teleopJoystickCommands.h>
#include <ros_control_boilerplate/JoystickState.h>

// Hold timing stats - accumulated total delay 
// and count of number of samples for messages
// with a given sequence number
struct Stats
{
	// Init values to zero in default constructor -
	// allows push_back(Stats()) to add a zeroed
	// record to the list
	Stats() :
		total_delay_(0),
		count_(0)
	{
	}
	double total_delay_;
	size_t count_;
};

void JSHeaderCallback(const std_msgs::Header &msg)
{
	static std::vector<Stats> stats;

	// Make sure there is enough space to hold
	// the stats for this sequence number
	while(msg.seq >= stats.size())
		stats.push_back(Stats());

	Stats &this_stat = stats[msg.seq];

	// If we've accumulated 100 samples from any
	// of the stats, print out the results stored
	// since the last printout
	this_stat.count_++;
	if (this_stat.count_ == 100)
	{
		for (size_t i = 0; i < stats.size(); i++)
			ROS_INFO_STREAM("delay = " << (stats[i].total_delay_ / stats[i].count_) << " seq = " << i);
		stats.clear();
	}
	else 
	{ 
		// Otherwise add the delay from this msg to the total
		// delay for messages sharing that sequence
		this_stat.total_delay_ += ros::Time::now().toSec() - msg.stamp.toSec();
	}
}

void HWHeaderCallback(const ros_control_boilerplate::JoystickState &msg)
{
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
		average_delay += (ros::Time::now().toSec() - msg.header.stamp.toSec()) / 100.0;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "debug_cmd_vel");
	ros::NodeHandle n;

	ros::Subscriber JS_subscriber = n.subscribe("test_header", 50, &JSHeaderCallback); //subscribing to teleopJoystickCommands loop
	ros::Subscriber hw_subscriber = n.subscribe("joystick_states", 5, &HWHeaderCallback); //subscribing to frcrobot_hw_interface loop

	ros::spin();
	return 0;
}
