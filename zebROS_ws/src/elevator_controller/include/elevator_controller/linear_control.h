#pragma once
#include <array>
#include <atomic>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <elevator_controller/arm_limiting.h>
#include <elevator_controller/CubeState.h>
#include <elevator_controller/ElevatorControl.h>
#include <elevator_controller/ElevatorControlS.h>
#include <elevator_controller/Intake.h>
#include <elevator_controller/ReturnElevatorCmd.h>

#include <talon_controllers/talon_controller_interface.h>

//#include <ros_control_boilerplate/MatchSpecificData.h>

#include <Eigen/Dense>

namespace elevator_controller
{

class ElevatorController
        : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface,
																hardware_interface::JointStateInterface,
																hardware_interface::PositionJointInterface>
{
	public:
		ElevatorController();
		bool init(hardware_interface::RobotHW *hw,
				ros::NodeHandle &root_nh,
				ros::NodeHandle &controller_nh);

		void update(const ros::Time &time, const ros::Duration &period);

		void starting(const ros::Time &time);

	private:
		double after_shift_max_accel_;		
		double after_shift_max_vel_;		
		//double cut_off_line_;
		double before_shift_max_accel_;		
		double before_shift_max_vel_;		

		
		bool intake_up_last_;
		double transition_time_;

		std::string name_;
		hardware_interface::JointStateHandle line_break_intake_high_;
		hardware_interface::JointStateHandle line_break_intake_low_;
		hardware_interface::JointStateHandle line_break_clamp_;
		std::atomic<bool> shift_cmd_;
		bool shifted_;
		std::atomic<double> clamp_cmd_;
		double climb_height_;
		std::atomic<bool> end_game_deploy_cmd_;
		bool end_game_deploy_t1_;
		bool end_game_deploy_t2_;
		double end_game_deploy_start_;

		double max_extension_;
		double min_extension_;
		std::atomic<double> intake_down_time_;

		double hook_depth_;
		double hook_min_height_;
		double hook_max_height_;

		struct IntakeCommand //This struct is highly subject to change
		{			
			double up_command;
			int32_t spring_command;
			double power;
			double down_time;
			IntakeCommand() : up_command(-1.0), spring_command(0.0), power(0.0), down_time(0.0) {}
		};

		talon_controllers::TalonMotionMagicCloseLoopControllerInterface lift_joint_;
		talon_controllers::TalonMotionMagicCloseLoopControllerInterface pivot_joint_;
		talon_controllers::TalonPercentOutputControllerInterface intake1_joint_;
		talon_controllers::TalonPercentOutputControllerInterface intake2_joint_;

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
		ros::Subscriber sub_stop_arm_;
		ros::ServiceServer service_command_;
		realtime_tools::RealtimeBuffer<IntakeCommand> intake_command_;
		ros::ServiceServer service_intake_;
		ros::ServiceServer service_clamp_;
		ros::ServiceServer service_shift_;
		ros::ServiceServer service_end_game_deploy_;
		//TODO: considering adding x offset?

		hardware_interface::JointHandle Clamp_;
		hardware_interface::JointHandle Shift_;
		hardware_interface::JointHandle EndGameDeploy_;

		hardware_interface::JointHandle IntakeUp_;
		hardware_interface::JointHandle IntakeHardSpring_;
		hardware_interface::JointHandle IntakeSoftSpring_;

		ros::Publisher CubeState_; 
		hardware_interface::JointHandle CubeStateJoint_;

		ros::Publisher ReturnCmd_; 
		ros::Publisher ReturnTrueSetpoint_; 

		ros::Publisher Odom_; 

		double arm_length_;
		double pivot_offset_;
		double lift_offset_;
		void cmdPosCallback(const elevator_controller::ElevatorControl& command);
		void stopCallback(const std_msgs::Bool& command);
		//void enabledCallback(const ros_control_boilerplate::MatchSpecificData& enabled);
		void lineBreakCallback(const sensor_msgs::JointState&);
		bool cmdPosService(elevator_controller::ElevatorControlS::Request &command, elevator_controller::ElevatorControlS::Response &res);
		bool intakeService(elevator_controller::Intake::Request &command, elevator_controller::Intake::Response &res);
		bool clampService(std_srvs::SetBool::Request &command, std_srvs::SetBool::Response &res); 
		bool shiftService(std_srvs::SetBool::Request &command, std_srvs::SetBool::Response &res); 
		bool endGameDeployService(std_srvs::Empty::Request &command, std_srvs::Empty::Response &res); 

		std::shared_ptr<arm_limiting::arm_limits> arm_limiter_;

		bool getFirstString(XmlRpc::XmlRpcValue value, std::string &str);

		double intake_power_diff_multiplier_;

		double f_arm_mass_;
		double f_arm_fric_;

		double f_lift_high_;
		double f_lift_low_;

		double last_tar_l;
		double last_tar_p;

		double norm_cur_lim_;
		double climb_cur_lim_;

		std::atomic<bool> stop_arm_;	

		//Something for getting the soft limit bounding boxes
		//some function for making limits based on soft limit bounding box
};//Class
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase);

}//Namespace
