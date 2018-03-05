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
//elevator_controller/cmd_pos
//elevator_controller/intake?
static double intake_config_x;
static double intake_config_y;
static double intake_low_x;
static double intake_low_y;

class autoAction {
    protected:
        //ros::NodeHandle nh_;
        //ros::NodeHandle nh_params(nh_, "teleop_params");
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<behaviors::IntakeLiftAction> as_;
        std::string action_name_;
        behaviors::IntakeLiftFeedback feedback_;
        behaviors::IntakeLiftResult result_;
        ros::Publisher Elevator;
        ros::ServiceClient IntakeSrv;
        ros::ServiceClient ElevatorSrv;
        ros::Publisher Clamp; 
		std::atomic<double> targetPosX;
		std::atomic<double> targetPosY;
		std::atomic<bool> high;
		std::atomic<bool> low;
		std::atomic<bool> success;
		std::atomic<int> goal_num;

    public:
		// TODO : pass in nh via the constructor - 
		// use n from main()?
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name),
			goal_num(-1)
        {
            as_.start();
            Elevator = nh_.advertise<elevator_controller::ElevatorControl>("elevator_controller/cmd_pos", 1);
            //Intake = nh_.advertise<elevator_controller::Intake>("elevator_controller/intake", 1);
            ElevatorSrv = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
            IntakeSrv = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
            Clamp = nh_.advertise<std_msgs::Bool>("elevator_controller/clamp", 1);
            elevator_odom = nh_.subscribe("/frcrobot/elevator_controller/odom", 1, &autoAction::OdomCallback, this);
            CubeState = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeCallback, this);
            HighCube = nh_.subscribe("/frcrobot/elevator_controller/high_cube", 1, &autoAction::highCubeCallback, this);
            // message_filters::Subscriber<std_msgs::Bool> cube_sub(nh_, "/frcrobot/elevator_controller/cube_state", 5);
            //message_filters::Subscriber<std_msgs::Bool> high_cube(nh_, "/frcrobot/elevator_controller/high_cube", 5);

            //typedef message_filters::sync_policies::ApproximateTime<std_msgs::Bool, std_msgs::Bool> cube_sync;
            //message_filters::Synchronizer<cube_sync> sync(cube_sync(5), cube_sub, high_cube);
            //sync.registerCallback(boost::bind(&autoAction::evaluateCubeState,this, _1, _2));
		}

        ros::Subscriber elevator_odom;// = nh_.subscribe("/frcrobot/elevator_controller/odom", 1, &OdomCallback);
        ros::Subscriber CubeState;
        ros::Subscriber HighCube;
    ~autoAction(void) 
    {
    }

	// TODO : goal should be to get as many variables as
	// locals as possible.  This includes goal_num and success
	// TODO: if the various goals are mutually exclusive, re-do
	// them as a single enumerated type.  This could then be
	// used in place of goal_num throughout the callback?
    void executeCB(const behaviors::IntakeLiftGoalConstPtr &goal) {
        ros::Rate r(10);
        const double startTime = ros::Time::now().toSec();
        success.store(false, std::memory_order_relaxed);
        while(success != true && (ros::Time::now().toSec()-startTime) < 15) {
            //TODO: use input up/down state
            if(as_.isPreemptRequested()) {
                ROS_WARN("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success.store(false, std::memory_order_relaxed);
                break;
            }
			// TODO : check ot make sure this won't spam
			// service calls.  Might want to move it ouside the loop -
			// kick off an action once and then inside the loop
			// poll for it to complete?
            if(goal->IntakeCube) {
                goal_num.store(0, std::memory_order_relaxed);
                elevator_controller::ElevatorControlS srv_elevator;
                srv_elevator.request.x = intake_config_x;
                srv_elevator.request.y = intake_config_y;
                srv_elevator.request.up_or_down = false;
                srv_elevator.request.override_pos_limits = false;
                ElevatorSrv.call(srv_elevator);
                elevator_controller::Intake srv;
                srv.request.up = false;
            }
            else if(goal->IntakeCubeNoLift) {
                goal_num.store(10, std::memory_order_relaxed);
                elevator_controller::Intake srv;
                srv.request.power = 0.8;
                srv.request.spring_state = 2; //soft in
                srv.request.up = false;
                IntakeSrv.call(srv);
            }
            else if(goal->GoToHeight) {
                goal_num.store(1, std::memory_order_relaxed);
                elevator_controller::ElevatorControlS srv;
                targetPosX.store(goal->x, std::memory_order_relaxed);
                targetPosY.store(goal->y, std::memory_order_relaxed);
                srv.request.x = goal->x;
                srv.request.y = goal->y;
                srv.request.up_or_down = goal->up_or_down;
                srv.request.override_pos_limits = goal->override_pos_limits;
                ElevatorSrv.call(srv);
            }
            else if(goal->MoveArmAway) {
                goal_num.store(2, std::memory_order_relaxed);
                elevator_controller::ElevatorControlS srv;
                srv.request.x = goal->x - .1; //TODO
                srv.request.y = goal->y +.2; //TODO
                srv.request.up_or_down = true;
                srv.request.override_pos_limits = true;
                srv.request.up_or_down = true;
                ElevatorSrv.call(srv);
                ros::Duration(.4).sleep(); //Dirty
                srv.request.x = goal->x - .1;
                srv.request.y = goal->y;
                srv.request.up_or_down = false;
                ros::Duration(.4).sleep(); //Dirty

                srv.request.x = intake_config_x;
                srv.request.y = intake_config_y;
                srv.request.up_or_down = false;
                ElevatorSrv.call(srv);
                success.store(true, std::memory_order_relaxed);
            } else {
				goal_num.store(-1, std::memory_order_relaxed);
				success.store(true, std::memory_order_relaxed);
			}
			// TODO : perhaps set a high_needed/low_needed
			// when the action is kicked off, then only check
			// high or low if the corresponding needed is set.
			// This way high and low can be set unconditionally
			// in the callback and goal_num can be made a local
			// var here.
            if(low) {
                if(goal_num != 10) {
                    elevator_controller::ElevatorControlS srv;
                    srv.request.x = intake_low_x;
                    srv.request.y = intake_low_y;
                    ElevatorSrv.call(srv);
                }
                ros::Duration(.2).sleep();
                success.store(true, std::memory_order_relaxed);
                low.store(false, std::memory_order_relaxed);
                high.store(false, std::memory_order_relaxed);
            }
            if(high) {
                success.store(true, std::memory_order_relaxed);
                low.store(false, std::memory_order_relaxed);
                high.store(false, std::memory_order_relaxed);
            }
            /*
            else if(goal->PlaceCube) {
                std_ElevatorMsgs::Bool ClampMsg;
                ClampMsg->data = false;
                Clamp.publish(ClampMsg);
            }
            */
			if (!success) {
				r.sleep();
				ros::spinOnce();
			}
        }
        //goal_num.store(-1, std::memory_order_relaxed);
        if(goal->IntakeCube || goal->IntakeCubeNoLift) {
            elevator_controller::Intake srv;
            srv.request.power = 0;
            IntakeSrv.call(srv);
			goal_num.store(-1, std::memory_order_relaxed);
				
        }
        result_.data = 1;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        return;
    }
	// TODO : rename to lowCubeCallback to match similar
	// highCubeCallback below?
	// Find a way to remove goal_num from this method
    void cubeCallback(const std_msgs::Bool &msg) {
		if(goal_num.load(std::memory_order_relaxed) == -1){
			return;
		}
        const bool cube_state = msg.data;
		static int cube_state_true = 0;
        if(cube_state) {
            cube_state_true += 1;
        }
		else {
			cube_state_true = 0;
		}
        if(cube_state_true >= 3) {
            low.store(true, std::memory_order_relaxed);
            cube_state_true = 0;
        }
    }
	// TODO : verfiy debounce code copied here from
	// cubeCallback works
    void highCubeCallback(const std_msgs::Bool &msg) {
		const int  local_goal_num = goal_num.load(std::memory_order_relaxed);
		if(local_goal_num == -1){
			return;
		}
        const bool cube_state = msg.data;
		static int cube_state_true = 0;
        if(cube_state) {
            cube_state_true += 1;
        }
		else {
			cube_state_true = 0;
		}
        if((cube_state_true >= 3) && (local_goal_num == 0 || local_goal_num == 10)) {
            high.store(true, std::memory_order_relaxed);
        }
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
		// TODO : change this to update x & y values and write them to
		// a current elevator pos struct. Then read this struct in the main
		// loop and update success accordingly
		// Should use a realtime_tools::RealtimeBuffer for the
		// communication. This will make sure accesses to x and y
		// are done as a unit - i.e. the x value matches the y
        if(goal_num.load(std::memory_order_relaxed) == 1 && fabs(msg->x-targetPosX.load(std::memory_order_relaxed)) < .1 && fabs(msg->y - targetPosY.load(std::memory_order_relaxed)) < .1) {
            success.store(true, std::memory_order_relaxed);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_interpreter_server");
    autoAction auto_action("auto_interpreter_server");
    ros::NodeHandle n;
    ros::NodeHandle n_params(n, "teleop_params");

	// If n is passed into autoAction class constructor, 
	// these reads could move there also and the vars they
	// read into moved to member vars
    if (n_params.getParam("intake_config_x", intake_config_x))
		ROS_ERROR("Could not read intake_config_x");
    if (n_params.getParam("intake_config_y", intake_config_y))
		ROS_ERROR("Could not read intake_config_y");
    if (n_params.getParam("intake_config_low_x", intake_low_x))
		ROS_ERROR("Could not read intake_config_low_x");
    if (n_params.getParam("intake_config_low_y", intake_low_y))
		ROS_ERROR("Could not read intake_config_low_y");
    
    ros::spin();

    return 0;
}
