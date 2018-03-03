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
		std::atomic<double> elevator_pos_x;
		std::atomic<double> elevator_pos_y;
		std::atomic<bool> high;
		std::atomic<bool> low;
		std::atomic<bool> success;
		std::atomic<int> goal_num;

    public:
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

    void executeCB(const behaviors::IntakeLiftGoalConstPtr &goal) {
        ros::Rate r(10);
        double startTime = ros::Time::now().toSec();
        success = false;
        while(success != true && (ros::Time::now().toSec()-startTime) < 15) {
            //TODO: use input up/down state
            if(as_.isPreemptRequested()) {
                ROS_WARN("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }
            if(goal->IntakeCube) {
                goal_num = 0;
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
                goal_num = 10;
                elevator_controller::Intake srv;
                srv.request.power = 0.8;
                srv.request.spring_state = 2; //soft in
                srv.request.up = false;
                IntakeSrv.call(srv);
            }
            else if(goal->GoToHeight) {
                goal_num = 1;
                elevator_controller::ElevatorControlS srv;
                targetPosX = goal->x;
                targetPosY = goal->y;
                srv.request.x = targetPosX;
                srv.request.y = targetPosY;
                srv.request.up_or_down = goal->up_or_down;
                srv.request.override_pos_limits = true;
                ElevatorSrv.call(srv);
            }
            else if(goal->MoveArmAway) {
                goal_num = 2;
                elevator_controller::ElevatorControlS srv;
                srv.request.x = goal->x - .1; //TODO
                srv.request.y = goal->y +.2; //TODO
                srv.request.up_or_down = true;
                srv.request.override_pos_limits = true;
                srv.request.up_or_down = true;
                ElevatorSrv.call(srv);
                ros::Duration(.4).sleep();
                srv.request.x = goal->x - .1;
                srv.request.y = goal->y;
                srv.request.up_or_down = false;
                ros::Duration(.4).sleep();

                srv.request.x = intake_config_x;
                srv.request.y = intake_config_y;
                srv.request.up_or_down = false;
                ElevatorSrv.call(srv);
                success = true;
            } else {
				goal_num = -1;
				success = true;
			}
            if(low) {
                if(goal_num != 10) {
                    elevator_controller::ElevatorControlS srv;
                    srv.request.x = intake_low_x;
                    srv.request.y = intake_low_y;
                    ElevatorSrv.call(srv);
                }
                ros::Duration(.2).sleep();
                success = true;
                low = false;
                high = false;
            }
            if(high) {
                success = true;
                low = false;
                high = false;
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
        //goal_num = -1;
        if(goal->IntakeCube || goal->IntakeCubeNoLift) {
            elevator_controller::Intake srv;
            srv.request.power = 0;
            IntakeSrv.call(srv);
			goal_num = -1;
				
        }
        result_.data = 1;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
        return;
    }
    void cubeCallback(const std_msgs::Bool &msg) {
		if(goal_num==-1){
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
            low = true;
            cube_state_true = 0;
        }
    }
    void highCubeCallback(const std_msgs::Bool &msg) {
		if(goal_num==-1){
			return;
		}
        const bool high_cube_state = msg.data;
        if(high_cube_state && (goal_num == 0 || goal_num == 10)) {
            high = true;
        }
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
    n_params.getParam("intake_config_low_x", intake_low_x);
    n_params.getParam("intake_config_low_y", intake_low_y);
    


    ros::spin();
    


    return 0;
}
