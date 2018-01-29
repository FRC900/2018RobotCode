//make this file so include is uploaded to github for structure
//
#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <pluginlib/class_list_macros.h>
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include "teleop_joystick_control/RobotState.h"
#include "elevator_controller/ElevatorControl.h"

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

//#include <talon_swerve_drive_controller/odometry.h>
#include <array>
#include <memory>
#include <Eigen/Dense>


namespace elevator_controller
{

class ElevatorController
        : public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
        public:
                ElevatorController();
		bool init(hardware_interface::TalonCommandInterface *hw,
			  ros::NodeHandle &root_nh,
			  ros::NodeHandle &controller_nh);

		void update(const ros::Time &time, const ros::Duration &period);
		

		void starting(const ros::Time &time);

	private:
		std::string name_;
		bool if_cube_;
		bool clamp_cmd_;
		double intake_power_;

		double max_extension_;
		double min_extension_;
		
		struct IntakePistonCommand //This struct is highly subject to change
		{			
			bool pivot_up;
			double left_pressure;
			double right_pressure;
	
		};
		//ros::Publisher RobotStatePub;
		//elevator_controller::RobotState RobotStateMsg;

		talon_controllers::TalonMotionMagicCloseLoopControllerInterface lift_joint_;
		talon_controllers::TalonMotionMagicCloseLoopControllerInterface pivot_joint_;
		talon_controllers::TalonPercentOutputControllerInterface intake_joint_;

		struct Commands
                {
                        Eigen::Vector2d lin;
                        bool up_or_down;
                        ros::Time stamp;

                        Commands() : lin({0.0, 0.0}), up_or_down(true), stamp(0.0) {}
                };
		realtime_tools::RealtimeBuffer<Commands> command_;
                Commands command_struct_;
		ros::Subscriber sub_command_;
		double arm_length_;
		double pivot_offset_;
		double lift_offset_;
		void cmdPosCallback(const elevator_controller::ElevatorControl& command);
		void clampCallback(const std_msgs::Bool& command); 
		void intakePowerCallback(const std_msgs::Float64& power); 
		//Add Callback for intake pneumatics, probably needs to be a custom msg
		
		//TODO: add odometry		
		//void compOdometry(const ros::Time& time, const double inv_delta_t);
		void evaluateCubeState();
		//Something for getting the soft limit bounding boxes
		//some function for making limits based on soft limit bounding box


};//Class
}//Namespace
