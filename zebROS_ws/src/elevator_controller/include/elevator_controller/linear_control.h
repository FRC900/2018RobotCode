#pragma once
#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <pluginlib/class_list_macros.h>
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
//#include <teleop_joystick_control/RobotState.h>
#include <elevator_controller/ElevatorControl.h>
#include <elevator_controller/ElevatorControlS.h>
#include <elevator_controller/Intake.h>
#include <elevator_controller/bool_srv.h>
#include <elevator_controller/Blank.h>
#include <elevator_controller/ReturnElevatorCmd.h>
#include <elevator_controller/arm_limiting.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

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
		double after_shift_max_accel_;		
		double after_shift_max_vel_;		

		double before_shift_max_accel_;		
		double before_shift_max_vel_;		

		std::string name_;
		bool line_break_intake_;
		bool line_break_clamp_;
		int line_break_intake_index_;
		int line_break_clamp_index_;
		bool shift_cmd_;
		bool shifted_;
		double clamp_cmd_;
		double climb_height_;
		bool end_game_deploy_cmd_;
		bool end_game_deploy_t1_;
		bool end_game_deploy_t2_;
		double end_game_deploy_start_;

		double max_extension_;
		double min_extension_;
		double intake_down_time_;

		double hook_depth_;
		double hook_min_height_;
		double hook_max_height_;

		struct IntakeCommand //This struct is highly subject to change
		{			
			double up_command;
			int32_t spring_command;
			double power;
			IntakeCommand() : up_command(-1.0), spring_command(0.0), power(0.0) {}

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
			bool override_pos_limits;
			bool override_sensor_limits;			
			ros::Time stamp;

			Commands() : lin({0.0, 0.0}), up_or_down(true), stamp(0.0), override_pos_limits(false), override_sensor_limits(false) {}
		};
		realtime_tools::RealtimeBuffer<Commands> command_;
		Commands command_struct_;
		ros::Subscriber sub_command_;
		ros::Subscriber sub_joint_state_;
		ros::ServiceServer service_command_;
		IntakeCommand intake_struct_;
		ros::ServiceServer service_intake_;
		ros::ServiceServer service_clamp_;
		ros::ServiceServer service_shift_;
		ros::ServiceServer service_end_game_deploy_;
		//TODO: considering adding x offset?

		ros::Publisher Clamp_; 
		ros::Publisher EndGameDeploy_; 
		ros::Publisher Shift_; 

		ros::Publisher CubeState_; 

		ros::Publisher IntakeUp_; 
		ros::Publisher IntakeHardSpring_; 
		ros::Publisher IntakeSoftSpring_; 

		ros::Publisher ReturnCmd_; 

		ros::Publisher Odom_; 

		double arm_length_;
		double pivot_offset_;
		double lift_offset_;
		void cmdPosCallback(const elevator_controller::ElevatorControl& command);
		void lineBreakCallback(const sensor_msgs::JointState&);
		bool cmdPosService(elevator_controller::ElevatorControlS::Request &command, elevator_controller::ElevatorControlS::Response &res);
		bool intakeService(elevator_controller::Intake::Request &command, elevator_controller::Intake::Response &res);
		bool clampService(elevator_controller::bool_srv::Request &command, elevator_controller::bool_srv::Response &res); 
		bool shiftService(elevator_controller::bool_srv::Request &command, elevator_controller::bool_srv::Response &res); 
		bool endGameDeployService(elevator_controller::Blank::Request &command, elevator_controller::Blank::Response &res); 

		std::shared_ptr<arm_limiting::arm_limits> arm_limiter_;

		bool getFirstString(XmlRpc::XmlRpcValue value, std::string &str);

		double last_tar_l;
		double last_tar_p;
		

		//Something for getting the soft limit bounding boxes
		//some function for making limits based on soft limit bounding box
};//Class
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase);

}//Namespace
