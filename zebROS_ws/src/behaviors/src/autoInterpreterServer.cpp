#include "ros/ros.h"
#include "actionlib/server/simpmle_action_server.h"
#include "behaviors/IntakeLiftAction.h"
#include "elevator_controller/Intake.h"
#include "elevator_controller/ElevatorControl.msg"
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
        std::string action_name;
        behaviors::IntakeLiftFeedback feedback_:
        behaviors::IntakeLiftResult result_;

    public:
        autoAction(std::string name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
        {
            as_.start()
        }
        ros::Publisher Elevator = nh_.advertise<elevator_controller::ElevatorControl>("elevator_controller/cmd_pos", 1);
        ros::Publisher Intake = nh_.advertise<elevator_controller::Intake>("elevator_controller/Intake", 1);
        
        //ros:Subscriber Linebreak = nh_.subscribe("linebreakYAY", 1, checkIntakeLinebreak);
    ~autoAction(void) 
    {
    }

    void executeCB(const behaviors::IntakeLiftGoalConstPtr &goal) {
        bool success;
        ros::Rate r(10);
        while(success != true && linebreak != true) {
            success = false;
            if(IntakeCube) {
                elevator_controller::Intake msg;
                msg->up = false;
                msg->soft_in = true;
                msg->power = .8;
                Intake.publish(msg);
            }
            else if(PlaceCube) {
                elevator_controller::ElevatorControl msg;
                msg->x = goal->x;
                msg->y = goal->y;
                msg->up_or_down = false;
                if(height > 50) {
                    msg->up_or_down = true;
                }
                msg->override_pos_limits = false;
                msg->override_sensor_limits = false;
                Elevator.publish(msg);
            }
            r.sleepOnce();
            ros::spin();
        }
        result_.data = 1
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);
    }
}
