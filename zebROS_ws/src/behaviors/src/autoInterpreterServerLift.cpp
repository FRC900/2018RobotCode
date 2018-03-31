#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/LiftAction.h"
#include "elevator_controller/ElevatorControl.h"
#include "elevator_controller/ElevatorControlS.h"
#include "elevator_controller/ReturnElevatorCmd.h"
#include "std_msgs/Bool.h"
#include <ros/console.h>
#include "behaviors/LiftAction.h"
#include "realtime_tools/realtime_buffer.h"
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
        //ros::Publisher Elevator;
        ros::ServiceClient ElevatorSrv_;
        //ros::Publisher Clamp; 
		ros::Subscriber elevator_odom_;
		// Elevator odometry. Make this a struct so that
		// reads from it get data which is consistent - avoid
		// cases where X has been modified by the callback
		// but Y hasn't yet.
		struct ElevatorPos
		{
		    ElevatorPos():
			X_(0),
			Y_(0),
			UpOrDown_(false)
		    {
		    }
		    ElevatorPos(double X, double Y, bool UpOrDown) :
			X_(X),
			Y_(Y),
			UpOrDown_(UpOrDown)
		    {
		    }

		    double X_;
		    double Y_;
		    bool   UpOrDown_;
		};
		realtime_tools::RealtimeBuffer<ElevatorPos> elevator_odom;
    public:
        autoAction(const std::string &name) :
            as_(nh_, name, boost::bind(&autoAction::executeCB, this, _1), false),
            action_name_(name)
	{
	    as_.start();
	    std::map<std::string, std::string> service_connection_header;
	    service_connection_header["tcp_nodelay"] = "1";
	    ElevatorSrv_ = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
	    elevator_odom_ = nh_.subscribe("/frcrobot/elevator_controller/odom", 1, &autoAction::OdomCallback, this);
	}

	~autoAction(void) 
	{
	}

    void executeCB(const behaviors::LiftGoalConstPtr &goal) {
	ros::Rate r(10);
	const double startTime = ros::Time::now().toSec();
	bool aborted = false;
	bool timed_out = false;
	if(goal->GoToPos)
	{
	    //ROS_INFO("start of got to pos");
	    elevator_controller::ElevatorControlS srv_elevator;
	    srv_elevator.request.x = goal->x;
	    srv_elevator.request.y = goal->y;
	    srv_elevator.request.up_or_down = goal->up_or_down;
	    srv_elevator.request.put_cube_in_intake = goal->put_cube_in_intake;
	    srv_elevator.request.override_pos_limits = goal->override_pos_limits;
	    if(!ElevatorSrv_.call(srv_elevator)) ROS_ERROR("Srv elevator call failed from auto server lift");;
	    ros::spinOnce();
		bool success = false;
	    while(!aborted && !timed_out && !success)
	    {
		const ElevatorPos odom = *(elevator_odom.readFromRT());
		const double dx = goal->x - odom.X_;
		const double dy = goal->y - odom.Y_;
		success = hypot(dx, dy) < goal->dist_tolerance && 
		    (odom.UpOrDown_ == goal->up_or_down || (arm_length_ - goal->x)/2 < goal->dist_tolerance) && 
		    fabs(dx) < goal->x_tolerance &&
		    fabs(dy) < goal->y_tolerance;

		//ROS_INFO_STREAM("goal x: " << goal->x << " goal y: " << goal->y << " goal up_or_down: " << goal->up_or_down);

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
	behaviors::LiftResult result;
	result.timed_out = timed_out;
        as_.setSucceeded(result);
    }

    void OdomCallback(const elevator_controller::ReturnElevatorCmd::ConstPtr &msg) {
	elevator_odom.writeFromNonRT(ElevatorPos(msg->x, msg->y, msg->up_or_down));
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
