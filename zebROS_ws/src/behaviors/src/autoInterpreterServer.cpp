#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/IntakeLiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "std_msgs/Bool.h"
#include "cstdlib"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/ReturnElevatorCmd.h"
//elevator_controller/cmd_pos
//elevator_controller/intake?
bool cube_state = true;
static double intake_config_x;
static double intake_config_y;
static int goal_num = -1;

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
        double targetPosX;
        double targetPosY;
        bool success;

    public:
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
            Elevator = nh_.advertise<elevator_controller::ElevatorControl>("elevator_controller/cmd_pos", 1);
            //Intake = nh_.advertise<elevator_controller::Intake>("elevator_controller/intake", 1);
            ElevatorSrv = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS");
            IntakeSrv = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
            Clamp = nh_.advertise<std_msgs::Bool>("elevator_controller/clamp", 1);
            elevator_odom = nh_.subscribe("/frcrobot/elevator_controller/odom", 1, &autoAction::OdomCallback, this);
            CubeState = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeCallback, this);
	}

        
        ros::Subscriber elevator_odom;// = nh_.subscribe("/frcrobot/elevator_controller/odom", 1, &OdomCallback);
        ros::Subscriber CubeState;
    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::IntakeLiftGoalConstPtr &goal) {
        ros::Rate r(10);
        double startTime = ros::Time::now().toSec();
        success = false;
        while(success != true && (ros::Time::now().toSec()-startTime) < 15) {
            if(as_.isPreemptRequested() || !ros::ok()) {
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
                srv.request.spring_state = 2; //soft in
                srv.request.power = .8; //TODO
                IntakeSrv.call(srv);
            }
            else if(goal->GoToHeight) {
                goal_num = 1;
                elevator_controller::ElevatorControlS srv;
                targetPosX = goal->x;
                targetPosY = goal->y;
                srv.request.x = targetPosX;
                srv.request.y = targetPosY;
                srv.request.override_pos_limits = true;
                ElevatorSrv.call(srv);
            }
            else if(goal->MoveArmAway) {
                goal_num = 2;
                elevator_controller::ElevatorControlS srv;
                srv.request.x = goal->x - .1; //TODO
                srv.request.y = goal->y +.2; //TODO
                srv.request.override_pos_limits = true;
                srv.request.up_or_down = true;
                ElevatorSrv.call(srv);
                ros::Duration(.4).sleep();
                srv.request.x = intake_config_x;
                srv.request.y = intake_config_y;
                srv.request.up_or_down = false;
                ElevatorSrv.call(srv);
                success = true;
            }
            /*
            else if(goal->PlaceCube) {
                std_ElevatorMsgs::Bool ClampMsg;
                ClampMsg->data = false;
                Clamp.publish(ClampMsg);
            }
            */
            r.sleep();
            ros::spinOnce();
        }
        goal_num = -1;
        if(goal->IntakeCube) {
            elevator_controller::Intake srv;
            srv.request.power = 0;
            IntakeSrv.call(srv);
        }
        result_.data = 1;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
    }
    void cubeCallback(const std_msgs::Bool &msg) {
        cube_state = msg.data;
        if(cube_state && goal_num == 0) {
            success = true;
        }
    }
    void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
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
    


    ros::spin();
    


    return 0;
}
