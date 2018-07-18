#include "auto_preseason/auto_loop.h"

static ros::Publisher cmd_vel_pub;
static ros::Subscriber cube_state_sub;
static ros::Subscriber cube_detection_sub;
static ros::ServiceClient path_to_cube_srv;
static ros::ServiceClient turn_to_angle_srv;

//Turn slowly in a circle
bool recovery_mode() 
{
	geometry_msgs::Twist vel;
	vel.linear.x = 0;
	vel.linear.y = 0;
	vel.angular.x = 0.1;
	cmd_vel_pub.publish(vel);

	return true;	
}

//Turn around in circles a lot, then calls the pathing service
bool path_to_cube()
{
	while(!cube_state)
	{
		recovery_mode();
	}

	std_srvs::Empty empty;
	if(!path_to_cube_srv.call(empty)) //should probably modify this to drive a certain distance away from the cube. And also to work?
	{
		ROS_ERROR("Path to cube service call failed in auto_loop");
		return false;
	}
	else
		return true;
}

bool intake_cube()
{
	auto_preseason::Angle angle = cube_detection_msg.angle;
	if (!turn_to_angle_srv.call(angle))
	{
		ROS_ERROR("Turn to angle service call failed in auto_loop");
		return false;
	}
	//run intakes in and drive forward slowly
}

bool path_to_exchange()
{
	bool exchange_detected = false;
	while(!exchange_detected)
	{
		recovery_mode();
	}
	//detect exchange and path to it. might be a node in the exchange detection package that will provide a service we can call
	return true;
}

bool place_in_exchange()
{
	//face exchange box
	//run intakes out at a certain speed
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_loop");
	ros::NodeHandle n;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);
	cube_state_sub = n.subscribe("/frcrobot/elevator_controller/cube_state", 1, &cube_state_callback);
	cube_detection_sub = n.subscribe("/frcrobot/cube_detection/cube_detection_something", 1, &cube_detection_callback);
	path_to_cube_srv = n.serviceClient<std_srvs::Empty>("/frcrobot/path_to_cube/path_service", false, service_connection_header);
	turn_to_angle_srv = n.serviceClient<auto_preseason::Angle>("/frcrobot/auto_preseason/turn_service", false, service_connection_header);

	while(ros::ok()) {
		//face the stack of cubes
		//drive toward the lowest and closest cube
		//attempt to intake the cube. if fail, back up and repeat. if succeed...
		//face the exchange wall
		//locate and drive toward the exchange
		//attempt to place cube in the exchange. if fail, repeat from "drive toward cube". if succeed, repeat from beginning

		//assumptions: the robot will never leave the area between the wall and the switch
		//the robot will time out an attempt on a certain cube once it fails too many times
		//we have cube detection that can differentiate between cubes (or not!) and we have exchange detection
		//the gyro is reasonably accurate so we can say "face forward" or "face backward"
		
		bool locate_cube = true;
		int tries = 0;

		bool haz_cube = true;
		if(haz_cube)
		{
			path_to_exchange();
			place_in_exchange();
		}
		else
		{
			path_to_cube();
			while (tries<=4)
			{
				intake_cube();
				tries++;
			}
		}
	}
}

void cube_state_callback(const elevator_controller::CubeState &msg)
{
	cube_state = msg.intake_high;
}

void cube_detection_callback(const cube_detection::CubeDetection &msg)
{
	cube_detection_msg = msg;
}
