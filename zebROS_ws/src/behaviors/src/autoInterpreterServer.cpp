#include "ros/ros.h"
#include <atomic>

#include <realtime_tools/realtime_buffer.h>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/RobotAction.h"
#include "behaviors/IntakeAction.h"
#include "behaviors/LiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/CubeState.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"

//elevator_controller/cmd_pos
//elevator_controller/intake?
static double intake_high_x;
static double delay_after_move_intake;
static double wait_after_clamp;
static double intake_high_y;
static bool intake_high_up_or_down;
static double intake_low_x;
static double intake_low_y;
static bool intake_low_up_or_down;
static double intake_ready_to_drop_y;
static double intake_ready_to_drop_x;
static bool intake_ready_to_drop_up_or_down;
static double drop_x_tolerance;
static double wait_open_before_drop;

class autoAction
{
	protected:
		//ros::NodeHandle nh_;
		//ros::NodeHandle nh_params(nh_, "teleop_params");
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::RobotAction> as_;
		std::string action_name_;
		behaviors::RobotFeedback feedback_;
		behaviors::RobotResult result_;
		ros::ServiceClient IntakeSrv_;
		ros::ServiceClient ElevatorSrv_;
		ros::ServiceClient ClampSrv_;
		std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> ai_;
		std::shared_ptr<actionlib::SimpleActionClient<behaviors::LiftAction>> al_;
		std::atomic<bool> high_cube_;

		// Elevator odometry. Make this a struct so that
		// reads from it get data which is consistent - avoid
		// cases where X has been modified by the callback
		// but Y hasn't yet.
		struct ElevatorPos
		{
			ElevatorPos():
				X_(0),
				Y_(0),
				UpOrDown_(false)
			{
			}
			ElevatorPos(double X, double Y, bool UpOrDown) :
				X_(X),
				Y_(Y),
				UpOrDown_(UpOrDown)
			{
			}

			double X_;
			double Y_;
			bool   UpOrDown_;
		};
		realtime_tools::RealtimeBuffer<ElevatorPos> elevator_odom;

		ros::Subscriber HighCube_;

	public:
		// TODO : pass in nh via the constructor -
		// use n from main()?
		autoAction(std::string name) :
			as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
			action_name_(name)
		{
			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";
			ElevatorSrv_ = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
			IntakeSrv_ = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
			ClampSrv_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
			HighCube_ = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeStateCallback, this);
			al_ = std::make_shared<actionlib::SimpleActionClient<behaviors::LiftAction>>("auto_interpreter_server_lift", true);
			ai_ = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("auto_interpreter_server_intake", true);
			as_.start();
		}

		~autoAction(void)
		{
		}

		void executeCB(const behaviors::RobotGoalConstPtr &goal)
		{
			ros::Rate r(10);
			const double startTime = ros::Time::now().toSec();
			bool aborted = false;
			bool success = false;
			bool timed_out = false;

			//if(!al_->getState().isDone())
				//al_->cancelAllGoals();
			//if(!ai_->getState().isDone())
				//ai_->cancelAllGoals();

			const double odom_x = elevator_odom.readFromRT()->X_;
			if (goal->IntakeCube)
			{
				ROS_INFO("start of pickup cube");
				std_srvs::SetBool srv_clamp;
				ros::spinOnce();
				behaviors::IntakeGoal goal_i;
				goal_i.IntakeCube = true;
				goal_i.time_out = 15;
				ai_->sendGoal(goal_i);
				srv_clamp.request.data = false;
				if (!ClampSrv_.call(srv_clamp)) ROS_ERROR("Srv_ clamp call failed");
				//If we aren't yet ready to drop, go to where we can drop
				behaviors::LiftGoal goal_l;
				goal_l.time_out = 5; //This honestly doesn't need to be an action lib
				goal_l.GoToPos = true;
				goal_l.x = intake_ready_to_drop_x;
				goal_l.y = intake_ready_to_drop_y;
				goal_l.up_or_down = intake_ready_to_drop_up_or_down;
				goal_l.override_pos_limits = false;
				goal_l.dist_tolerance = 1.0;
				goal_l.y_tolerance = 1.0;
				goal_l.x_tolerance = drop_x_tolerance;

				al_->sendGoal(goal_l);
				//loop till we get to where we can drop
				while (!aborted && !timed_out)
				{
					ROS_INFO("start of pickup cube 1");
					if (as_.isPreemptRequested() || !ros::ok())
					{
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						aborted = true;
						break;
					}
					if (!aborted)
					{
						r.sleep();
						ros::spinOnce();
						timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;
						//time_out if the action times out
						if (ai_->getState().isDone())
						{
							timed_out = timed_out || ai_->getResult()->timed_out;
							break;
						}
						if (al_->getState().isDone())
						{
							timed_out = timed_out || al_->getResult()->timed_out;
						}
					}
				}
				double t_before_move_intake = 0;
				if (!aborted && !timed_out)
				{
					elevator_controller::Intake srvIntake;
					srvIntake.request.power = 0;
					srvIntake.request.up = true;
					srvIntake.request.spring_state = 1; //hard_out
					if (!IntakeSrv_.call(srvIntake)) ROS_ERROR("Srv_ intake call failed");;
					t_before_move_intake = ros::Time::now().toSec();
				}

				while (!aborted && !timed_out)
				{
					ROS_INFO("start of pickup cube 2");
					if (as_.isPreemptRequested() || !ros::ok())
					{
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						aborted = true;
						break;
					}
					if (!aborted)
					{
						r.sleep();
						ros::spinOnce();
						timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;
						if (al_->getState().isDone())
						{
							timed_out = timed_out || (al_->getResult()->timed_out);
							if(ros::Time::now().toSec() - t_before_move_intake > delay_after_move_intake)
								break;
						}
					}
				}
				if (!aborted && !timed_out && !high_cube_)
				{
					behaviors::LiftGoal goal_l;
					goal_l.time_out = 5;
					goal_l.GoToPos = true;
					goal_l.x = intake_low_x;
					goal_l.y = intake_low_y;
					goal_l.up_or_down = intake_low_up_or_down;
					goal_l.override_pos_limits = false;
					goal_l.dist_tolerance = 0.1; //Tolerances are intentionally large, will require testing
					goal_l.y_tolerance = 0.1;
					goal_l.x_tolerance = 0.1;
					al_->sendGoal(goal_l);
					while (!aborted && !timed_out)
					{
						ROS_INFO("start of pickup cube 3");
						if (as_.isPreemptRequested() || !ros::ok())
						{
							ROS_WARN("%s: Preempted", action_name_.c_str());
							as_.setPreempted();
							aborted = true;
							break;
						}
						if (!aborted)
						{
							r.sleep();
							ros::spinOnce();
							timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;

							if (al_->getState().isDone())
							{
								//time_out if the action times out
								timed_out = timed_out || (al_->getResult()->timed_out);
								break;
							}

						}
					}
				}
				else if (!aborted && !timed_out)
				{
					behaviors::LiftGoal goal_l;
					goal_l.time_out = 5;
					goal_l.GoToPos = true;
					goal_l.x = intake_high_x;
					goal_l.y = intake_high_y;
					goal_l.up_or_down = intake_high_up_or_down;
					goal_l.override_pos_limits = false;
					goal_l.dist_tolerance = 0.1; //Tolerances are intentionally large, will require testing
					goal_l.y_tolerance = 0.1;
					goal_l.x_tolerance = 0.1;
					al_->sendGoal(goal_l);
					while (!aborted && !timed_out)
					{
						ROS_INFO("start of pickup cube 4");
						if (as_.isPreemptRequested() || !ros::ok())
						{
							ROS_WARN("%s: Preempted", action_name_.c_str());
							as_.setPreempted();
							aborted = true;
							break;
						}
						if (!aborted)
						{
							r.sleep();
							ros::spinOnce();
							timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;

							if (al_->getState().isDone())
							{
								//time_out if the action times out
								timed_out = timed_out || (al_->getResult()->timed_out);
								break;
							}

						}
					}
				}
					
				double clamp_time;
				if (!aborted && !timed_out)
				{
					srv_clamp.request.data = true;
					clamp_time = ros::Time::now().toSec();
					if (!ClampSrv_.call(srv_clamp)) ROS_ERROR("Srv_ clamp call failed");;

				}
				while (!aborted && !timed_out)
				{
					ROS_INFO("start of pickup cube 5");
					if (as_.isPreemptRequested() || !ros::ok())
					{
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						aborted = true;
						break;
					}
					if (ros::Time::now().toSec() - clamp_time > wait_after_clamp)
					{
						behaviors::LiftGoal goal_l;
						goal_l.time_out = 5;
						goal_l.GoToPos = true;
						//go to specified after position
						goal_l.x = goal->x;
						goal_l.y = goal->y;
						goal_l.up_or_down =  goal->up_or_down;
						goal_l.override_pos_limits = goal->override_pos_limits;
						goal_l.dist_tolerance = goal->dist_tolerance;
						goal_l.y_tolerance = goal->y_tolerance;
						goal_l.x_tolerance = goal->x_tolerance;
						al_->sendGoal(goal_l);
						break;
					}
					if (!aborted)
					{
						r.sleep();
						ros::spinOnce();
						timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;
					}
				}
				while (!aborted && !timed_out)
				{
					ROS_INFO("start of pickup cube 6");
					if (as_.isPreemptRequested() || !ros::ok())
					{
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						aborted = true;
						break;
					}
					if (!aborted)
					{
						r.sleep();
						ros::spinOnce();
						timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;

						if (al_->getState().isDone())
						{
							//time_out if the action times out
							timed_out = timed_out || (al_->getResult()->timed_out);
							break;
						}

					}
				}
			}
			if (goal->MoveToIntakeConfig)
			{
				ai_->cancelAllGoals();
				ROS_INFO("start of go to intake config with cube");
				behaviors::LiftGoal goal_l;
				goal_l.time_out = 5; //This honestly doesn't need to be an action lib
				goal_l.GoToPos = true;
				goal_l.x = intake_ready_to_drop_x;
				goal_l.y = intake_ready_to_drop_y;
				goal_l.up_or_down = intake_ready_to_drop_up_or_down;
				goal_l.override_pos_limits = false;
				goal_l.dist_tolerance = 1.0;
				goal_l.y_tolerance = 1.0;
				goal_l.x_tolerance = drop_x_tolerance;

				al_->sendGoal(goal_l);
				
				double t_before_move_intake = 0;
				elevator_controller::Intake srvIntake;
				srvIntake.request.power = 0;
				srvIntake.request.up = true;
				srvIntake.request.spring_state = 1; //hard_out
				if (!IntakeSrv_.call(srvIntake)) ROS_ERROR("Srv_ intake call failed");;
				t_before_move_intake = ros::Time::now().toSec();
				
				//loop till we get to where we can drop
				while (!aborted && !timed_out)
				{
					if (as_.isPreemptRequested() || !ros::ok())
					{
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						aborted = true;
						break;
					}
					if (!aborted)
					{
						r.sleep();
						ros::spinOnce();
						timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;
						//time_out if the action times out
						if (ros::Time::now().toSec() - t_before_move_intake > wait_open_before_drop)
						{
							break;
						}
						if (al_->getState().isDone())
						{
							timed_out = timed_out || al_->getResult()->timed_out;
						}
					}
				}
				if(!aborted && !timed_out)
				{
					behaviors::LiftGoal goal_l;
					goal_l.time_out = 5; 
					goal_l.GoToPos = true;
					goal_l.x = intake_high_x;
					goal_l.y = intake_high_y;
					goal_l.up_or_down = intake_high_up_or_down;
					goal_l.override_pos_limits = false;
					goal_l.dist_tolerance = 0.1;
					goal_l.y_tolerance = 0.1;
					goal_l.x_tolerance = 0.1;

					al_->sendGoal(goal_l);
					
					elevator_controller::Intake srvIntake;
					srvIntake.request.power = .15; //hold dat cube
					srvIntake.request.up = false;
					srvIntake.request.spring_state = 3; //hard_in
					if (!IntakeSrv_.call(srvIntake)) ROS_ERROR("Srv_ intake call failed");;
					
				}
				while (!aborted && !timed_out)
				{
					if (as_.isPreemptRequested() || !ros::ok())
					{
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						aborted = true;
						break;
					}
					if (!aborted)
					{
						r.sleep();
						ros::spinOnce();
						timed_out = timed_out || (ros::Time::now().toSec() - startTime) > goal->time_out;
						//time_out if the action times out
						if (al_->getState().isDone())
						{
							timed_out = timed_out || al_->getResult()->timed_out;
							break;
						}
					}
				}
			}
			if (timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if (!aborted)
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}
			else
			{
				ROS_INFO("%s: Aborted", action_name_.c_str());
			}
			//if(!al_->getState().isDone())
				al_->cancelAllGoals();
			//if(!ai_->getState().isDone())
				ai_->cancelAllGoals();
			result_.timed_out = timed_out;
			as_.setSucceeded(result_);

			return;
		}

		// TODO : debounce code?
		void cubeStateCallback(const elevator_controller::CubeState &msg)
		{
			high_cube_ = msg.intake_high;
		}

		void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg)
		{
			elevator_odom.writeFromNonRT(ElevatorPos(msg->x, msg->y, msg->up_or_down));
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_interpreter_server");
	autoAction auto_action("auto_interpreter_server");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");

	if (!n_params.getParam("intake_high_up_or_down", intake_high_up_or_down))
		ROS_ERROR("Could not read intake_high_up_or_down in auto server");
	if (!n_params.getParam("intake_low_up_or_down", intake_low_up_or_down))
		ROS_ERROR("Could not read intake_low_up_or_down in auto server");
	if (!n_params.getParam("intake_ready_to_drop_x", intake_ready_to_drop_x))
		ROS_ERROR("Could not read intake_ready_to_drop_x");
	if (!n_params.getParam("intake_ready_to_drop_y", intake_ready_to_drop_y))
		ROS_ERROR("Could not read intake_ready_to_drop_y");
	if (!n_params.getParam("intake_ready_to_drop_up_or_down", intake_ready_to_drop_up_or_down))
		ROS_ERROR("Could not read intake_ready_to_drop_up_or_down in auto server");
	if (!n_params.getParam("wait_after_clamp", wait_after_clamp))
		ROS_ERROR("could not read wait_after_clamp");
	if (!n_params.getParam("delay_after_move_intake", delay_after_move_intake))
		ROS_ERROR("could not read delay_after_move_intake");

	// If n is passed into autoAction class constructor,
	// these reads could move there also and the vars they
	// read into moved to member vars
	if (!n_params.getParam("intake_high_x", intake_high_x))
		ROS_ERROR_STREAM("Could not read intake_high_x: ");
	if (!n_params.getParam("intake_high_y", intake_high_y))
		ROS_ERROR("Could not read intake_high_y");
	if (!n_params.getParam("intake_low_x", intake_low_x))
		ROS_ERROR("Could not read intake_low_x");
	if (!n_params.getParam("intake_low_y", intake_low_y))
		ROS_ERROR("Could not read intake_low_y");
	if (!n_params.getParam("wait_open_before_drop", wait_open_before_drop))
		ROS_ERROR("Could not read wait_open_before_drop");
	if (!n_params.getParam("drop_x_tolerance", drop_x_tolerance))
		ROS_ERROR("Could not read drop_x_tolerance");

	ros::spin();

	return 0;
}
