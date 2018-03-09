#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/IntakeAction.h"
#include "elevator_controller/Intake.h"
#include "std_msgs/Bool.h"
#include <cstdlib>
#include <atomic>
#include <ros/console.h>

static double intake_power;
static double intake_hold_power;
class autoAction {
    protected:
        //ros::NodeHandle nh_;
        //ros::NodeHandle nh_params(nh_, "teleop_params");
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
        std::string action_name_;
        ros::ServiceClient IntakeSrv;
	std::atomic<double> time_out;
	std::atomic<int> cube_state_true;
	std::atomic<bool> success;
	std::atomic<bool> aborted;
	bool timed_out;
	behaviors::IntakeResult result_;
    public:
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
            IntakeSrv = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
            CubeState = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeCallback, this);
	}
	ros::Subscriber CubeState;

    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::IntakeGoalConstPtr &goal) {
        ros::Rate r(10);
        double startTime = ros::Time::now().toSec();
        success = false;
	timed_out = false;
	aborted = false;
        if(goal->IntakeCube) 
	{
		cube_state_true = 0;
                elevator_controller::Intake srv;
                srv.request.power = intake_power;
                srv.request.spring_state = 2; //soft in
                srv.request.up = false;
                if(!IntakeSrv.call(srv)) ROS_ERROR("Srv intake call failed in auto interpreter server intake");;
		ros::spinOnce();
		while(!success && !timed_out && !aborted) {
		    
		    success = cube_state_true > 2; 
		    if(as_.isPreemptRequested() || !ros::ok()) {
			ROS_WARN("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			aborted = true;
			break;
		    }
			if (!aborted) {
				r.sleep();
				ros::spinOnce();
				timed_out = (ros::Time::now().toSec()-startTime) > goal->time_out;
			}
			/*
			else
			{
				srv.request.power = -intake_hold_power;
				srv.request.spring_state = 3; //hard in
				srv.request.up = false;
                		if(!IntakeSrv.call(srv)) ROS_ERROR("Srv intake call failed in auto interpreter server intake");;
			}
			*/		    
		}
	}
	//else if goal->
	//{}
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
    void cubeCallback(const std_msgs::Bool &msg) {
        if(msg.data) {
            cube_state_true += 1;
        }
	else {
		cube_state_true = 0;
	}
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_interpreter_server_intake");
    autoAction auto_action("auto_interpreter_server_intake");
    
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");

    n_params.getParam("intake_power", intake_power);
    n_params.getParam("intake_hold_power", intake_hold_power);


    ros::spin();
    return 0;
}
