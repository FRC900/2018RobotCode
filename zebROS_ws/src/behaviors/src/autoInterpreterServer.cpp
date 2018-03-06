#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/IntakeLiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "std_msgs/Bool.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include <cstdlib>
#include <atomic>
#include <ros/console.h>
//elevator_controller/cmd_pos
//elevator_controller/intake?

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
	std::atomic<bool> high_cube;
	behaviors::IntakeGoal i_goal; 
	behaviors::LiftGoal l_goal;
	double intake_config_x;	 
	double wait_after_clamp;	 
	double intake_config_y;	
	double intake_config_up_or_down;	
	double intake_low_x;	 
	double intake_low_y;	 
	double intake_low_up_or_down;	
	double intake_ready_to_drop_y;
	double intake_ready_to_drop_x;
	double intake_ready_to_drop_up_or_down; 
    public:
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name),
			goal_num(-1)
        {
            as_.start();
            ElevatorSrv = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
            IntakeSrv = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
            ClampSrv= nh_.serviceClient<elevator_controller::bool_srv>("/frcrobot/elevator_controller/intake");
            HighCube = nh_.subscribe("/frcrobot/elevator_controller/high_cube", 1, &autoAction::highCubeCallback, this);
	}

        ros::Subscriber HighCube;
    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::IntakeLiftGoalConstPtr &goal) {
	ros::Rate r(10);
        double startTime = ros::Time::now().toSec();
        aborted = false;
        success = false;
	timed_out = false;
	if(goal->IntakeCube)
	{
		
		elevator_controller::bool_srv srv_clamp;
		ros::spinOnce();
		goal_i.IntakeCube = true;
		goal_i.time_out = 15;
		ai->sendGoal(goal_i);
		srv_clamp.request.data = false;
		ClampSrv.call(srv_clamp);
		bool ready_to_drop = fabs(intake_ready_to_drop_x - odom_x) < drop_x_tolerance;	
		//If we aren't yet ready to drop, go to where we can drop
		if(!ready_to_drop)
		{
			goal_l.time_out = 5;
			goal_l.GoToHeight = true;
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
			    if(as_.isPreemptRequested()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
			    }
			    if (!aborted) {
				r.sleep();
				ros::spinOnce();
				timed_out |= (ros::Time::now().toSec()-startTime) < goal->time_out;
				

				//time_out if the action times out
				if(ai->getState().isDone())
				{
					timed_out |= *(ai->getResult());  
			    	}
				if(al->getState().isDone())
				{
					timed_out |= *(al->getResult());  
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
		    if(as_.isPreemptRequested()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
		    if (!aborted) {
			r.sleep();
			ros::spinOnce();
			timed_out |= (ros::Time::now().toSec()-startTime) < goal->time_out;
				
			if(al->getState().isDone())
			{
				//time_out if the action times out
				timed_out |= *(al->getResult());  
			}
			if(ai->getState().isDone())
			{
				timed_out |= *(ai->getResult());  
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
			    if(as_.isPreemptRequested()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
			    }
			    if (!aborted) {
				r.sleep();
				ros::spinOnce();
				timed_out |= (ros::Time::now().toSec()-startTime) < goal->time_out;
					
				if(al->getState().isDone())
				{
					//time_out if the action times out
					timed_out |= *(al->getResult());  
					break;
				}
			
		    }


		

		}
		}
		if(!aborted && !timed_out)
		{
			srv_clamp.request.data = true;
			double clamp_time = ros::Time::now().toSec();
			ClampSrv.call(srv_clamp);
			srvIntake.request.power = 0;
            		srvIntake.request.up = false;
            		srvIntake.request.spring_state = 1; //hard_out
			IntakeSrv.call(srvIntake);

		}
		while(!aborted && !timed_out)
		{
			if(as_.isPreemptRequested()) {
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
				timed_out |= (ros::Time::now().toSec()-startTime) < goal->time_out;
		        }
		}
		while(!aborted && !timed_out)
		{
		    if(as_.isPreemptRequested()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
		    if (!aborted) {
			r.sleep();
			ros::spinOnce();
			timed_out |= (ros::Time::now().toSec()-startTime) < goal->time_out;
				
			if(al->getState().isDone())
			{
				//time_out if the action times out
				timed_out |= *(al->getResult());  
				break;
			}
			
		    }


		}
	}	
	if(goal->MoveToExchangeConfig)
	{
		elevator_controller::bool_srv srv_clamp;
		ros::spinOnce();
		bool ready_to_drop = fabs(intake_ready_to_drop_x - odom_x) < drop_x_tolerance;	
		srvIntake.request.power = 0;
            	srvIntake.request.up = false;
            	srvIntake.request.spring_state = 1; //hard_out
		IntakeSrv.call(srvIntake);
		//If we aren't yet ready to drop, go to where we can drop
		if(!ready_to_drop)
		{
			goal_l.time_out = 5;
			goal_l.GoToHeight = true;
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
			    if(as_.isPreemptRequested()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				aborted = true;
				break;
			    }
			    if (!aborted) {
				r.sleep();
				ros::spinOnce();
				timed_out |= (ros::Time::now().toSec()-startTime) < goal->time_out;
				

				//time_out if the action times out
				if(ai->getState().isDone())
				{
					timed_out |= *(ai->getResult());  
			    	}
				if(al->getState().isDone())
				{
					timed_out |= *(al->getResult());  
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
		    if(as_.isPreemptRequested()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
		    if (!aborted) {
			r.sleep();
			ros::spinOnce();
			timed_out |= (ros::Time::now().toSec()-startTime) < goal->time_out;
				
			if(al->getState().isDone())
			{
				//time_out if the action times out
				timed_out |= *(al->getResult());  
				break;
			}
			
		    }


		}
		if(!aborted && !timed_out)
		{
			srvIntake.request.power = 0;
	    	        srvIntake.request.up = false;
		        srvIntake.request.spring_state = 2; //soft_in
		        IntakeSrv.call(srvIntake);
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
        as_.setSucceeded(timed_out);
 
        return;
    }
    void highCubeCallback(const std_msgs::Bool &msg) {
	high_cube = msg.data;    

    }
    /*
    void evaluateCubeState(std_msgs::Bool &cube, std_msgs::Bool &high_cube) {
        const bool cube_state = cube.data;
        const bool high_cube_state = high_cube.data;
        if(cube_state && high_cube_state) {
            high = true;
        }
        else if(cube_state) {
            low = true;
        }
        */
    //} 
    void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
        elevator_pos_x = msg->x;
        elevator_pos_y = msg->y;
        if(fabs(msg->x-targetPosX) < .1 && fabs(msg->y - targetPosY) < .1 && goal_num == 1) {
            success = true;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_interpreter_server");
    autoAction auto_action("auto_interpreter_server");
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");

    n_params.getParam("intake_config_x", intake_config_x);
    n_params.getParam("intake_config_y", intake_config_y);
    n_params.getParam("intake_config_up_or_down", intake_config_up_or_down);
    n_params.getParam("intake_config_low_x", intake_low_x);
    n_params.getParam("intake_config_low_y", intake_low_y);
    n_params.getParam("intake_config_low_up_or_down", intake_low_up_or_down);
    n_params.getParam("intake_ready_to_drop_x", intake_ready_to_drop_x);
    n_params.getParam("intake_ready_to_drop_y", intake_ready_to_drop_y);
    n_params.getParam("intake_ready_to_drop_up_or_down", intake_ready_to_drop_up_or_down);

    n_params.getParam("wait_after_clamp", wait_after_clamp);
    ros::spin();
    


    return 0;
}
