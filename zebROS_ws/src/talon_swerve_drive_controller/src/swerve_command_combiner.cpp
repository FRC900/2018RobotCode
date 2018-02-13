#include "ros/ros.h"
#include "ros/console.h"
#include "talon_swerve_drive_controller/CompleteCmd.h"
#include "geometry_msgs/TwistStamped.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ros/time.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <math.h>



static ros::Publisher combined_msg;

void combine(const geometry_msgs::TwistStamped::ConstPtr &cmd_vel, const trajectory_msgs::JointTrajectory::ConstPtr &points)
{
	talon_swerve_drive_controller::CompleteCmd holder;
	if(fabs(cmd_vel->header.stamp.toSec()- points->header.stamp.toSec()) < .05)
	{
		holder.twist_ = cmd_vel->twist;
		holder.cmd_vel_or_points = true;
	}
	else if(cmd_vel->header.stamp.toSec() < points->header.stamp.toSec())
	{
		holder.twist_ = cmd_vel->twist;
		holder.cmd_vel_or_points = true;
	}
	else
	{
		holder.joint_trajectory = *(points);
		holder.cmd_vel_or_points = false;
	}
	combined_msg.publish(holder);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "swerve_drive_parser");
    ros::NodeHandle n;

    message_filters::Subscriber<geometry_msgs::TwistStamped> cmd_vel_sub(n, "/frcrobot/swerve_drive_controller/cmd_vel", 3);
    message_filters::Subscriber<trajectory_msgs::JointTrajectory> points_sub(n, "/frcrobot/swerve_drive_controller/trajectory_points", 3);
    combined_msg = n.advertise<talon_swerve_drive_controller::CompleteCmd>("combined_cmd", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, trajectory_msgs::JointTrajectory> CommandSync;
    message_filters::Synchronizer<CommandSync> sync(CommandSync(5), cmd_vel_sub, points_sub);
    sync.registerCallback(boost::bind(&combine, _1, _2));


    ros::spin();

    return 0;
}
