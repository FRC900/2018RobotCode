#include "ros/ros.h"
#include "ros/console.h"
#include <control_msgs/JointTrajectoryControllerState.h>
#include <talon_swerve_drive_controller/CompleteCmd.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>
#include <vector>
#include <std_msgs/Bool.h>

static ros::Publisher points_publisher;
static bool run_;
static std::vector<trajectory_msgs::JointTrajectoryPoint> points_;	
void combine(const control_msgs::JointTrajectoryControllerState &point) 
{
    points_.push_back(point.desired);
   



}
void callRun(const std_msgs::Bool &run) 
{
	run_ = run.data;
    if(run_)
    {
    	trajectory_msgs::JointTrajectory holder;
   	holder.points = points_;
	holder.header.stamp = ros::Time::now(); 
	points_publisher.publish(holder);
	points_.clear();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "point_combiner");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/frcrobot/base_trajectory_controller/state", 1, combine);
    ros::Subscriber sub_ = n.subscribe("testing/run", 1, callRun);

    points_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/frcrobot/swerve_drive_controller/trajectory_points", 1);

    ros::spin();

    return 0;
}
