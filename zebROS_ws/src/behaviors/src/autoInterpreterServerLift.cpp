#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/LiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "std_msgs/Bool.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include <cstdlib>
#include <atomic>
#include <ros/console.h>
#include "behaviors/LiftAction.h"
//elevator_controller/cmd_pos
//elevator_controller/intake?
static double arm_length_;

class autoAction {
    protected:
        //ros::NodeHandle nh_;
        //ros::NodeHandle nh_params(nh_, "teleop_params");
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<behaviors::LiftAction> as_;
        std::string action_name_;
        ros::Publisher Elevator;
        ros::ServiceClient IntakeSrv;
        ros::ServiceClient ElevatorSrv;
        ros::Publisher Clamp; 
	std::atomic<bool> success;
	std::atomic<bool> aborted;
	std::atomic<double> odom_x;
	std::atomic<double> odom_y;
	std::atomic<double> odom_up_or_down;
	bool timed_out;
	behaviors::LiftResult result_;
    public:
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
            ElevatorSrv = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
            elevator_odom = nh_.subscribe("/frcrobot/elevator_controller/odom", 1, &autoAction::OdomCallback, this);
		}

        ros::Subscriber elevator_odom;// = nh_.subscribe("/frcrobot/elevator_controller/odom", 1, &OdomCallback);
    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::LiftGoalConstPtr &goal) {
	ros::Rate r(10);
        double startTime = ros::Time::now().toSec();
        success = false;
        aborted = false;
	timed_out = false;
	if(goal->GoToPos)
	{
		elevator_controller::ElevatorControlS srv_elevator;
                srv_elevator.request.x = goal->x;
                srv_elevator.request.y = goal->y;
                srv_elevator.request.up_or_down = goal->up_or_down;
                srv_elevator.request.override_pos_limits = goal->override_pos_limits;
                ElevatorSrv.call(srv_elevator);
                ros::spinOnce();
		while(!aborted && !timed_out)
		{
		    success = sqrt((goal->x - odom_x) * (goal->x - odom_x) + (goal->y - odom_y) * 
		    (goal->y - odom_y)) < goal->dist_tolerance && (odom_up_or_down == goal->up_or_down  
		    || (arm_length_ - goal->x)/2 < goal->dist_tolerance) && fabs(goal->x - odom_x) < goal->
		    x_tolerance &&  fabs(goal->y - odom_y) < goal->y_tolerance;
		    if(as_.isPreemptRequested() || !ros::ok()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
		    if (!success) {
			r.sleep();
			ros::spinOnce();
                        timed_out = (ros::Time::now().toSec()-startTime) > goal->time_out;      
		    }
		}



	}
	//else if() ....
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
    void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
        odom_x = msg->x;
        odom_y = msg->y;
    	odom_up_or_down = msg->up_or_down;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_interpreter_server_lift");
    autoAction auto_action("auto_interpreter_server_lift");

    ros::NodeHandle n;

    ros::NodeHandle n_params(n, "elevator_controller");
    if (!n_params.getParam("arm_length", arm_length_))
                ROS_ERROR("Could not read arm_length");

    ros::spin();
    
    return 0;
}
