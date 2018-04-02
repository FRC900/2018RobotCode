#include "robot_visualizer/profile_follow.h"

robot_visualizer::ProfileFollower::Request local_req;
ros::ServiceServer follow_srv;
ros::Subscriber talon_sub;
ros::Publisher robot_state_pub;
bool msg_recieved = false;

int index_talon = -1;

bool running;
int slot_run;
double time_remaining;
std::vector<int> remaining_points;
robot_visualizer::RobotVisualizeState state_msg;

double x_offset = 0.5955;
double y_offset = 4.59;
double theta_offset = M_PI/2;


int main(int argc, char **argv) {
    ros::init(argc, argv, "profile_follow");
    ros::NodeHandle n;

	follow_srv = n.advertiseService("/frcrobot/visualize_auto", &follow_service);
	talon_sub = n.subscribe("/frcrobot/talon_states", 1, &talon_cb);
	robot_state_pub = n.advertise<robot_visualizer::RobotVisualizeState>("/frcrobot/robot_viz_state", 1);


	ros::Rate rate(50);
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
		if(!msg_recieved) {continue;}
		if(!(running && slot_run >= local_req.start_id)) {continue;}

		//ROS_WARN("5");
		//ROS_INFO_STREAM("running?: " << running << " slot: " << slot_run << " start_id: " << local_req.start_id << " traj size: " << local_req.joint_trajectories.size()); 


		//ROS_INFO_STREAM(" indexing at: " << local_req.joint_trajectories[slot_run - local_req.start_id].points.size() - remaining_points[slot_run] - 1 << " remaining points: " << remaining_points[slot_run] << " total_points: " <<  local_req.joint_trajectories[slot_run - local_req.start_id].points.size());
		
		state_msg.x = x_offset + local_req.joint_trajectories[slot_run - local_req.start_id].points[local_req.joint_trajectories[slot_run - local_req.start_id].points.size() - remaining_points[slot_run] - 1 ].positions[1];

		//ROS_WARN("1");
		state_msg.y = y_offset - local_req.joint_trajectories[slot_run - local_req.start_id].points[local_req.joint_trajectories[slot_run - local_req.start_id].points.size() - remaining_points[slot_run] - 1  ].positions[0];
		//ROS_WARN("2");

		state_msg.theta = theta_offset + local_req.joint_trajectories[slot_run - local_req.start_id].points[local_req.joint_trajectories[slot_run - local_req.start_id].points.size() - remaining_points[slot_run] - 1 ].positions[2];
		//ROS_WARN("3");
		

		state_msg.arm_pos = 1; 
		state_msg.intake_pos = 0; //TODO fix these

		//ROS_WARN("4");
		robot_state_pub.publish(state_msg);
		
	}
};

bool follow_service(robot_visualizer::ProfileFollower::Request &req, robot_visualizer::ProfileFollower::Response &res)
{
	msg_recieved = true;
	ROS_ERROR_STREAM("srv_Called_real with size: "<< req.joint_trajectories.size());
	
	local_req = req;	
	return true;
}

void talon_cb(const talon_state_controller::TalonState &msg)
{


	if(index_talon == -1)
	{ 
		for(size_t i = 0; i < msg.can_id.size(); i++)
		{
			ROS_INFO_STREAM("id: " << msg.can_id[i]);
			if(msg.can_id[i] == 14)
			{		
				index_talon = i;
				break;
			}
		}
		if(index_talon == -1)
		{
			ROS_ERROR("id 14 talon not found");
			return;
		}
	}
	running = msg.custom_profile_status[index_talon].running;
	slot_run = msg.custom_profile_status[index_talon].slotRunning;
	time_remaining = msg.custom_profile_status[index_talon].slotRunning;
	remaining_points = msg.custom_profile_status[index_talon].remainingPoints;
}



