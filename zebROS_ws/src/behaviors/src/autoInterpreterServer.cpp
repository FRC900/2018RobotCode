#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/IntakeLiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/Intake.h"
#include "std_msgs/Bool.h"
#include "cstdlib"
//elevator_controller/cmd_pos
//elevator_controller/intake?
bool linebreak = false;
void checkIntakeLinebreak(void) {
    linebreak = true;
}

class autoAction {
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<behaviors::IntakeLiftAction> as_;
        std::string action_name_;
        behaviors::IntakeLiftFeedback feedback_;
        behaviors::IntakeLiftResult result_;
        ros::Publisher Elevator;
        ros::ServiceClient IntakeSrv;
        ros::Publisher Clamp; 

    public:
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start();
            Elevator = nh_.advertise<elevator_controller::ElevatorControl>("elevator_controller/cmd_pos", 1);
            //Intake = nh_.advertise<elevator_controller::Intake>("elevator_controller/intake", 1);
            IntakeSrv = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake");
            Clamp = nh_.advertise<std_msgs::Bool>("elevator_controller/clamp", 1);
        }

        
        //ros:Subscriber Linebreak = nh_.subscribe("linebreakYAY", 1, checkIntakeLinebreak);
    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::IntakeLiftGoalConstPtr &goal) {
        bool success;
        ros::Rate r(20);
        double startTime = ros::Time::now().toSec();
        success = false;
        while(success != true && linebreak != true && ros::Time::now().toSec()-startTime < 15) {
            if(goal->IntakeCube) {
                elevator_controller::Intake srv;
                srv.request.up = false;
                srv.request.spring_state = 2; //soft in
                srv.request.power = .8; //TODO
                IntakeSrv.call(srv);
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
        elevator_controller::Intake srv;
        srv.request.power = 0;
        IntakeSrv.call(srv);
        result_.data = 1;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_Interpreter_Server");
    autoAction auto_action("auto_Interpreter_Server");
    ros::spin();
    
    return 0;
}
