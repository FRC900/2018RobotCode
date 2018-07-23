#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/IntakeAction.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/CubeState.h"
#include "std_msgs/Bool.h"
#include <atomic>
#include <ros/console.h>

static double intake_power;
static double intake_hold_power;
double linebreak_debounce_iterations; 

//#Important Changes

class autoAction {
    protected:
        //ros::NodeHandle nh_;
        //ros::NodeHandle nh_params(nh_, "teleop_params");
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
        std::string action_name_;
        ros::ServiceClient IntakeSrv_;
	std::atomic<int> cube_state_true;
	behaviors::IntakeResult result_;
	ros::Subscriber CubeState_;
	ros::Subscriber Proceed_;
	bool proceed;
    public:
        autoAction(const std::string &name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";
            IntakeSrv_ = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
            CubeState_ = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeCallback, this);
			Proceed_ = nh_.subscribe("/frcrobot/auto_interpreter_server/proceed", 1, &autoAction::proceedCallback, this);


	}

    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::IntakeGoalConstPtr &goal) {
        ros::Rate r(10);
        double startTime = ros::Time::now().toSec();
        bool timed_out = false;
        bool aborted = false;
        if(goal->IntakeCube) {
			//ROS_INFO("start of intake cube");
            cube_state_true = 0;
            elevator_controller::Intake srv;
            srv.request.power = intake_power;
            srv.request.other_power = intake_power;
            srv.request.just_override_power = false;
            srv.request.spring_state = 2; //soft in
            srv.request.up = false;
            if(!IntakeSrv_.call(srv)) 
                ROS_ERROR("Srv intake call failed in auto interpreter server intake");
            //else
                //ROS_INFO("Srv intake call OK in auto interpreter server intake");
            ros::spinOnce();
			bool success = false;
            while(!success && !timed_out && !aborted) {
                success = cube_state_true > linebreak_debounce_iterations && (proceed  || !goal->wait_to_proceed); 
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
                        if(!IntakeSrv_.call(srv)) ROS_ERROR("Srv intake call failed in auto interpreter server intake");;
            }
            */		    
            }
            srv.request.power = success ? 0.15 : 0;
            srv.request.other_power = success ? 0.15 : 0;
            srv.request.spring_state = 3; //soft in
            srv.request.up = false;
            srv.request.just_override_power = !success;
            if(!IntakeSrv_.call(srv)) 
                ROS_ERROR("Srv intake call failed in auto interpreter server intake");
            //else
                //ROS_INFO("Srv intake call OK in auto interpreter server intake");
		}
		else
		{
            cube_state_true = 0;
            elevator_controller::Intake srv;
            srv.request.power = -1;
            srv.request.other_power = -1;
            srv.request.just_override_power = false;
            srv.request.spring_state = 2; //soft in
            srv.request.up = false;
            if(!IntakeSrv_.call(srv)) 
                ROS_ERROR("Srv intake call failed in auto interpreter server intake");
            //else
                //ROS_INFO("Srv intake call OK in auto interpreter server intake");
            ros::spinOnce();
			bool success = false;
            while(!success && !timed_out && !aborted) {
                success = cube_state_true > linebreak_debounce_iterations && (proceed  || !goal->wait_to_proceed); 
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
                        if(!IntakeSrv_.call(srv)) ROS_ERROR("Srv intake call failed in auto interpreter server intake");;
            }
            */		    
            }
            srv.request.power = success ? 0.15 : 0;
            srv.request.other_power = success ? 0.15 : 0;
            srv.request.spring_state = 3; //soft in
            srv.request.up = false;
            srv.request.just_override_power = !success;
            if(!IntakeSrv_.call(srv)) 
                ROS_ERROR("Srv intake call failed in auto interpreter server intake");
            //else
                //ROS_INFO("Srv intake call OK in auto interpreter server intake");
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
    void cubeCallback(const elevator_controller::CubeState &msg) {
        if(msg.intake_low || msg.intake_high) {
            cube_state_true += 1;
        }
	else {
		cube_state_true = 0;
    }
	}
	void proceedCallback(const std_msgs::Bool &msg)
	{
		proceed = msg.data;
	}

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_interpreter_server_intake");
    autoAction auto_action("auto_interpreter_server_intake");
    
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");
    ros::NodeHandle n_auto_interpreter_server_intake_params(n, "auto_interpreter_server_intake_params");

    if (!n_params.getParam("intake_power", intake_power))
		ROS_ERROR("Could not read intake_power in autoInterpreterServerIntake");

    if (!n_params.getParam("intake_hold_power", intake_hold_power))
		ROS_ERROR("Could not read intake_hold_power in autoInterpreterServerIntake");


    if (!n_auto_interpreter_server_intake_params.getParam("linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in autoInterpreterServerIntake");
    ros::spin();
    return 0;
}
