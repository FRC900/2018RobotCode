#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/RobotAction.h"
#include "behaviors/IntakeAction.h"
#include "behaviors/LiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/bool_srv.h"
#include "std_msgs/Bool.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include <cstdlib>
#include <atomic>
#include <ros/console.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

//elevator_controller/cmd_pos
//elevator_controller/intake?
static double intake_config_x;	 
static double wait_after_clamp;	 
static double intake_config_y;	
static double intake_config_up_or_down;	
static double intake_low_x;	 
static double intake_low_y;	 
static double intake_low_up_or_down;	
static double intake_ready_to_drop_y;
static double intake_ready_to_drop_x;
static double intake_ready_to_drop_up_or_down;
static double drop_x_tolerance;

class autoAction {
    protected:
        //ros::NodeHandle nh_;
        //ros::NodeHandle nh_params(nh_, "teleop_params");
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<behaviors::RobotAction> as_;
        std::string action_name_;
        behaviors::RobotFeedback feedback_;
        behaviors::RobotResult result_;
        ros::ServiceClient IntakeSrv;
        ros::ServiceClient ElevatorSrv;
        ros::ServiceClient ClampSrv; 
	std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> ai;
	std::shared_ptr<actionlib::SimpleActionClient<behaviors::LiftAction>> al;
	std::atomic<bool> success;
	std::atomic<bool> aborted;
	std::atomic<bool> high_cube;
	elevator_controller::Intake srvIntake;
	behaviors::IntakeGoal goal_i; 
	behaviors::LiftGoal goal_l;
	std::atomic<double> odom_x;
        std::atomic<double> odom_y;
        std::atomic<double> odom_up_or_down;
	bool timed_out;
	
    public:
		// TODO : pass in nh via the constructor - 
		// use n from main()?
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
        {
			as_.start();
			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";
			ElevatorSrv = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", true, service_connection_header);
			IntakeSrv = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", true, service_connection_header);
            ClampSrv= nh_.serviceClient<elevator_controller::bool_srv>("/frcrobot/elevator_controller/clamp", true, service_connection_header);
            HighCube = nh_.subscribe("/frcrobot/elevator_controller/high_cube", 1, &autoAction::highCubeCallback, this);
    		al = std::make_shared<actionlib::SimpleActionClient<behaviors::LiftAction>>("auto_interpreter_server_lift", true);
    		ai = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("auto_interpreter_server_intake", true);
	}

        ros::Subscriber HighCube;
    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::RobotGoalConstPtr &goal) {
	ros::Rate r(10);
        double startTime = ros::Time::now().toSec();
        aborted = false;
        success = false;
	timed_out = false;

	if((goal->IntakeCube))
	{
		
		elevator_controller::bool_srv srv_clamp;
		ros::spinOnce();
		goal_i.IntakeCube = true;
		goal_i.time_out = 15;
		ai->sendGoal(goal_i);
		srv_clamp.request.data = false;
		if(!ClampSrv.call(srv_clamp)) ROS_ERROR("Srv clamp call failed");
		bool ready_to_drop = fabs(intake_ready_to_drop_x - odom_x) < drop_x_tolerance;	
		//If we aren't yet ready to drop, go to where we can drop
		if(!ready_to_drop)
		{
			goal_l.time_out = 5;
			goal_l.GoToPos = true;
			goal_l.x = intake_ready_to_drop_x;
			goal_l.y = intake_ready_to_drop_y;
			goal_l.up_or_down = intake_ready_to_drop_up_or_down;
			goal_l.override_pos_limits = false;
			goal_l.dist_tolerance = 1.0;
			goal_l.y_tolerance = 1.0;
			goal_l.x_tolerance = drop_x_tolerance;

			al->sendGoal(goal_l);
			//loop till we get to where we can drop
			while(!aborted && !timed_out && !ready_to_drop )
			{
			    if(as_.isPreemptRequested() || !ros::ok()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
			    }
			    if (!aborted) {
				r.sleep();
				ros::spinOnce();
				timed_out |= (ros::Time::now().toSec()-startTime) > goal->time_out;
				

				//time_out if the action times out
				if(ai->getState().isDone())
				{
					timed_out |= (ai->getResult()->timed_out);  
			    	}
				if(al->getState().isDone())
				{
					timed_out |= (al->getResult()->timed_out);  
			    		break;
				}
			    }
			}
		
		}
		if(!aborted && !timed_out)
		{
			goal_l.time_out = 5;
			goal_l.GoToPos = true;
			goal_l.x = intake_config_x;
			goal_l.y = intake_config_y;
			goal_l.up_or_down = intake_config_up_or_down;
			goal_l.override_pos_limits = true;
			goal_l.dist_tolerance = 0.1; //Tolerances are intentionally large, will require testing
			goal_l.y_tolerance = 0.1;
			goal_l.x_tolerance = 0.1;
			al->sendGoal(goal_l);
		}
		while(!aborted && !timed_out)
		{
		    if(as_.isPreemptRequested() || !ros::ok()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
		    if (!aborted) {
			r.sleep();
			ros::spinOnce();
			timed_out |= (ros::Time::now().toSec()-startTime) > goal->time_out;
				
			if(al->getState().isDone())
			{
				//time_out if the action times out
				timed_out |= (al->getResult()->timed_out);  
			}
			if(ai->getState().isDone())
			{
				timed_out |= (ai->getResult()->timed_out);  
			}
			if(ai->getState().isDone() &&	ai->getState().isDone())
			{
				break;
			}
			
		    }


		}
		if(!aborted && !timed_out && !high_cube)
		{
			
			goal_l.time_out = 5;
			goal_l.GoToPos = true;
			goal_l.x = intake_low_x;
			goal_l.y = intake_low_y;
			goal_l.up_or_down = intake_low_up_or_down;
			goal_l.override_pos_limits = true;
			goal_l.dist_tolerance = 0.1; //Tolerances are intentionally large, will require testing
			goal_l.y_tolerance = 0.02;
			goal_l.x_tolerance = 0.1;
			al->sendGoal(goal_l);
			while(!aborted && !timed_out)
			{
			    if(as_.isPreemptRequested() || !ros::ok()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
			    }
			    if (!aborted) {
	// TODO : goal should be to get as many variables as
	// locals as possible.  This includes goal_num and success
	// TODO: if the various goals are mutually exclusive, re-do
	// them as a single enumerated type.  This could then be
	// used in place of goal_num throughout the callback?
				r.sleep();
				ros::spinOnce();
				timed_out |= (ros::Time::now().toSec()-startTime) > goal->time_out;
					
				if(al->getState().isDone())
				{
					//time_out if the action times out
					timed_out |= (al->getResult()->timed_out);  
					break;
				}
			
		    	    }


		

			}
		}
		double clamp_time;
		if(!aborted && !timed_out)
		{
			srv_clamp.request.data = true;
			clamp_time = ros::Time::now().toSec();
			if(!ClampSrv.call(srv_clamp)) ROS_ERROR("Srv clamp call failed");;
			srvIntake.request.power = 0;
            		srvIntake.request.up = false;
            		srvIntake.request.spring_state = 1; //hard_out
			if(!IntakeSrv.call(srvIntake)) ROS_ERROR("Srv intake call failed");;

		}
		while(!aborted && !timed_out)
		{
			if(as_.isPreemptRequested() || !ros::ok()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
		    	}
			if(ros::Time::now().toSec() - clamp_time > wait_after_clamp)
			{
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
				al->sendGoal(goal_l);
				break;
			}
			if (!aborted) {
				r.sleep();
				ros::spinOnce();
				timed_out |= (ros::Time::now().toSec()-startTime) > goal->time_out;
		        }
		}
		while(!aborted && !timed_out)
		{
		    if(as_.isPreemptRequested() || !ros::ok()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
		    if (!aborted) {
			r.sleep();
			ros::spinOnce();
			timed_out |= (ros::Time::now().toSec()-startTime) > goal->time_out;
				
			if(al->getState().isDone())
			{
				//time_out if the action times out
				timed_out |= (al->getResult()->timed_out);  
				break;
			}
			
		    }


		}
	}	
	if(goal->MoveToIntakeConfig)
	{
		elevator_controller::bool_srv srv_clamp;
		ros::spinOnce();
		bool ready_to_drop = fabs(intake_ready_to_drop_x - odom_x) < drop_x_tolerance;	
		srvIntake.request.power = 0;
            	srvIntake.request.up = false;
            	srvIntake.request.spring_state = goal->hasCube ? 1 : 2; //hard_out or soft in depending
		if(!IntakeSrv.call(srvIntake)) ROS_ERROR("Srv intake call failed");;
		//If we aren't yet ready to drop, go to where we can drop
		if(!ready_to_drop)
		{
			goal_l.time_out = 5;
			goal_l.GoToPos = true;
			goal_l.x = intake_ready_to_drop_x;
			goal_l.y = intake_ready_to_drop_y;
			goal_l.up_or_down = intake_ready_to_drop_up_or_down;
			goal_l.override_pos_limits = false;
			goal_l.dist_tolerance = 1.0;
			goal_l.y_tolerance = 1.0;
			goal_l.x_tolerance = drop_x_tolerance;
			
			al->sendGoal(goal_l);
			//loop till we get to where we can drop
			while(!aborted && !timed_out && !ready_to_drop )
			{
			    if(as_.isPreemptRequested() || !ros::ok()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
			    }
			    if (!aborted) {
				r.sleep();
				ros::spinOnce();
				timed_out |= (ros::Time::now().toSec()-startTime) > goal->time_out;
				

				//time_out if the action times out
				if(al->getState().isDone())
				{
					timed_out |= (al->getResult()->timed_out);  
			    		break;
				}
			    }
			}
		
		}
		if(!aborted && !timed_out)
		{
			goal_l.time_out = 5;
			goal_l.GoToPos = true;
			goal_l.x = intake_config_x;
			goal_l.y = intake_config_y;
			goal_l.up_or_down = intake_config_up_or_down;
			goal_l.override_pos_limits = true;
			goal_l.dist_tolerance = 0.1; //Tolerances are intentionally large, will require testing
			goal_l.y_tolerance = 0.1;
			goal_l.x_tolerance = 0.1;
			al->sendGoal(goal_l);
		}
		while(!aborted && !timed_out)
		{
		    if(as_.isPreemptRequested() || !ros::ok()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
		    if (!aborted) {
			r.sleep();
			ros::spinOnce();
			timed_out |= (ros::Time::now().toSec()-startTime) > goal->time_out;
				
			if(al->getState().isDone())
			{
				//time_out if the action times out
				timed_out |= (al->getResult()->timed_out);  
				break;
			}
			
		    }


		}
		if(!aborted && !timed_out)
		{
			srvIntake.request.power = 0;
	    	        srvIntake.request.up = false;
		        srvIntake.request.spring_state = 2; //soft_in
		        if(!IntakeSrv.call(srvIntake)) ROS_ERROR("Srv intake call failed");;
		}
	}	
        if(timed_out)
        {
                ROS_INFO("%s: Timed Out", action_name_.c_str());
        }
        else if(!aborted)
	{
                ROS_INFO("%s: Succeeded", action_name_.c_str());
        }
	else
	{
                ROS_INFO("%s: Aborted", action_name_.c_str());
	}
        result_.timed_out = timed_out;
	as_.setSucceeded(result_);
 
        return;
    }
	// TODO : verfiy debounce code copied here from
	// cubeCallback works
    void highCubeCallback(const std_msgs::Bool &msg) {
	high_cube = msg.data;    

    }
    /*
    void evaluateCubeState(std_msgs::Bool &cube, std_msgs::Bool &high_cube) {
        const bool cube_state = cube.data;
        const bool high_cube_state = high_cube.data;
        if(cube_state && high_cube_state) {
            high.store(true, std::memory_order_relaxed);
        }
        else if(cube_state) {
            low.store(true, std::memory_order_relaxed);
        }
        */
    //} 
    void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
        odom_x = msg->x;
        odom_y = msg->y;
        odom_up_or_down = msg->up_or_down;
		// TODO : change this to update x & y values and write them to
		// a current elevator pos struct. Then read this struct in the main
		// loop and update success accordingly
		// Should use a realtime_tools::RealtimeBuffer for the
		// communication. This will make sure accesses to x and y
		// are done as a unit - i.e. the x value matches the y
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_interpreter_server");
    autoAction auto_action("auto_interpreter_server");
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");

    n_params.getParam("intake_config_up_or_down", intake_config_up_or_down);
    n_params.getParam("intake_config_low_up_or_down", intake_low_up_or_down);
    n_params.getParam("intake_ready_to_drop_x", intake_ready_to_drop_x);
    n_params.getParam("intake_ready_to_drop_y", intake_ready_to_drop_y);
    n_params.getParam("intake_ready_to_drop_up_or_down", intake_ready_to_drop_up_or_down);

    n_params.getParam("wait_after_clamp", wait_after_clamp);
	// If n is passed into autoAction class constructor, 
	// these reads could move there also and the vars they
	// read into moved to member vars
    if (!n_params.getParam("intake_config_x", intake_config_x))
		ROS_ERROR_STREAM("Could not read intake_config_x: ");
    if (!n_params.getParam("intake_config_y", intake_config_y))
		ROS_ERROR("Could not read intake_config_y");
    if (!n_params.getParam("intake_config_low_x", intake_low_x))
		ROS_ERROR("Could not read intake_config_low_x");
    if (!n_params.getParam("intake_config_low_y", intake_low_y))
		ROS_ERROR("Could not read intake_config_low_y");
    



    ros::spin();

    return 0;
}
