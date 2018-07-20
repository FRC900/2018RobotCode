#include "auto_preseason/auto_loop.h"

static ros::Publisher cmd_vel_pub;
static ros::Subscriber cube_state_sub;
static ros::Subscriber cube_detection_sub;
static ros::ServiceClient path_to_cube_srv;
static ros::ServiceClient turn_to_angle_srv;
static ros::ServiceClient intake_srv;

bool haz_cube = false; //for simulation only.
float min_pathing_dist = 0.75; //meters

//Turn slowly in a circle
bool recovery_mode()//different for echange and cubes, include navX angle 
{
	ROS_INFO_NAMED("auto loop", "recovery_mode");
	geometry_msgs::Twist vel;
	vel.linear.x = 0;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0.1;
	cmd_vel_pub.publish(vel);

	return true;	
}

//Turn around in circles a lot, then calls the pathing service
bool path_to_cube()
{
	ROS_INFO_NAMED("auto loop", "path_to_cube");
	std_srvs::Empty empty;
	if(!path_to_cube_srv.call(empty)) //should probably modify this to drive a certain distance away from the cube. And also to work?
	{
		ROS_ERROR("Path to cube service call failed in auto_loop");
		//return false; //commented out for simulation
	}
	else
		return true;
}

bool intake_cube()
{
	ROS_INFO_NAMED("auto loop", "intake_cube");
	auto_preseason::Angle angle;
	angle.request.angle = cube_detection_msg.angle; //make sure this is from the same ref. point and that data is updated!
	if (!turn_to_angle_srv.call(angle))
	{
		ROS_ERROR("Turn to angle service call failed in auto_loop");
		//return false; //comment out for simulation
	}
	//run intakes in
	//figure out the actionlib stuffs for that actually
	
	geometry_msgs::Twist vel;
	vel.linear.x = 0.1;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	vel.angular.z = 0.0;
	cmd_vel_pub.publish(vel);

	haz_cube = true; //for simulation

	return true;
}

bool path_to_exchange()
{
	ROS_INFO_NAMED("auto loop", "path_to_exchange");
	bool exchange_detected = false; //for simulation
	while(!exchange_detected)
	{
		recovery_mode();
		exchange_detected = true; //for simulation
	}
	//detect exchange and path to it. might be a node in the exchange detection package that will provide a service we can call
	return true;
}

bool place_in_exchange()
{
	ROS_INFO_NAMED("auto loop", "place_in_exchange");
	//face exchange box
	//run intakes out at a certain speed
	haz_cube = false;
	return true;
}

bool start_drive()
{
	//do actionlib stuff so that the drive starts?!?
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_loop");
	ros::NodeHandle n;

	enum State {STARTING_STATE, HAZ_CUBE, CUBE_SEEN_CLOSE, CUBE_SEEN_FAR, NO_CUBE_FOUND, DRIVING_TO_EXCHANGE, PATHING_TO_CUBE};

	State state = STARTING_STATE;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);
	cube_state_sub = n.subscribe("/frcrobot/elevator_controller/cube_state", 1, &cube_state_callback);
	cube_detection_sub = n.subscribe("/frcrobot/cube_detection/cube_detection_something", 1, &cube_detection_callback);
	path_to_cube_srv = n.serviceClient<std_srvs::Empty>("/frcrobot/path_to_cube/path_service", false, service_connection_header);
	turn_to_angle_srv = n.serviceClient<auto_preseason::Angle>("/frcrobot/auto_preseason/turn_service", false, service_connection_header);
	intake_srv = n.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);

	while(ros::ok()) {
		//assumptions: the robot will never leave the area between the wall and the switch
		//the robot will time out an attempt on a certain cube once it fails too many times
		//we have cube detection that can differentiate between cubes (or not!) and we have exchange detection
		//the gyro is reasonably accurate so we can say "face forward" or "face backward"
		//we have two ZEDs -- one on the exchange side and one on the cube side. We run cube detection on both ZEDs and exchange detection on one.
		//
		//requirements for vision things:
		//it ONLY CAN DETECT ONE THING AT ONCE or the first is always the best

		cube_found = (cube_detection_msg.location.size() > 0);
		qr_found = (exchange_detection_msg.location.size() > 0); //replace with actual
		cube_dist = cube_detection_msg.location[0].z;

		switch(state)
		{
			case STARTING_STATE : if(haz_cube)
								  {
									  if(qr_found)
									  {
										  if(exchange_detection_msg.location[0].z < 1) //TODO: figure out which sensor is going to be used for distance to wall (ultrasonic)
											  state = HAZ_CUBE_FACING_EXCHANGE_CLOSE;
										  else
											  state = HAZ_CUBE_FACING_EXCHANGE_FAR;
									  }
									  else
										  state = HAZ_CUBE_NOT_FACING_EXCHANGE;
								  }
								  else if(!haz_cube && cube_dist >= min_pathing_dist && cube_found)
									  state = CUBE_SEEN_FAR;
								  else if(!haz_cube && cube_dist <= min_pathing_dist  && cube_found)
									  state = CUBE_SEEN_CLOSE;
								  else if(!cube_found)
									  state = NO_CUBE_FOUND;
			case HAZ_CUBE_FACING_EXCHANGE_FAR : 
								  start_drive(); //TODO: needs to be an actionlib thing -- can start and then run in the background
								  state = DRIVING_TO_EXCHANGE;
			case DRIVING_TO_EXCHANGE :
								  if(driving == finished) //TODO: fix this with actionlib things
									  state = HAZ_CUBE_FACING_EXCHANGE_CLOSE;
			case HAZ_CUBE_FACING_EXCHANGE_CLOSE :
								  start_vault_cube(); //TODO: some actionlib stuff
								  state = VAULTING_CUBE;
			case VAULTING_CUBE :
								  if(vaulting_finished) //TODO: some actionlib stuff
								  {
									  if(true) //TODO: figure out what sensor needs to determine this
										  state = NO_CUBE_FOUND; //has to ignore the cube that it's looking at
									  if(false)
										  state = CUBE_SEEN_CLOSE;
								  }
			case HAZ_CUBE_NOT_FACING_EXCHANGE :
								  recovery_mode();
								  if(exchange_found)
								  {
									  if(dist_from_exchange < 1)
										  state = HAZ_CUBE_FACING_EXCHANGE_CLOSE;
									  else
										  state = HAZ_CUBE_FACING_EXCHANGE_FAR;
								  }
			case CUBE_SEEN_FAR : 
								  start_path_to_cube(); //TODO: needs to be an actionlib thing -- can start and then run in the background
								  state = PATHING_TO_CUBE;
			case PATHING_TO_CUBE : 
								  if(pathing_finished) //TODO: check pathing sending back actionlib stuff
									  state = CUBE_SEEN_CLOSE;
			case CUBE_SEEN_CLOSE :
								  fix_angle_cube();
								  start_run_in_intake();
								  state = RUNNING_INTAKE;
			case RUNNING_INTAKE :
								  if(intake == finished)
								  {
									  if(exchange_found)
									  {
										  if(dist_from_exchange < 1)
											  state = HAZ_CUBE_FACING_EXCHANGE_CLOSE;
										  else
											  state = HAZ_CUBE_FACING_EXCHANGE_FAR;
									  }
									  else
										  state = HAZ_CUBE_NOT_FACING_EXCHANGE;
								  }
			case NO_CUBE_FOUND :
								  recovery_mode();
								  if(cube_found && cube_dist < 5)
									  state = CUBE_SEEN_CLOSE;
								  if(cube_found && cube_dist >= 5)
									  state = CUBE_SEEN_FAR;
									  
		}
	
									  
		/*	
		int tries = 0;
		if(!haz_cube && cube_dist >= 5 && cube_found)
		{
			path_to_cube();
		}
		else if(!haz_cube && cube_dist < 5 && cube_found)
		{
			while (tries<=4)
			{
				if(!intake_cube())
					tries++;
				else
					break;
			}
		}
		else if(!haz_cube && !cube_found)
		{
			recovery_mode();
		}
		else
		{
			if(path_to_exchange())
				place_in_exchange();
		}
		ROS_INFO_STREAM(haz_cube);
		*/
	}
}

void cube_state_callback(const elevator_controller::CubeState &msg)
{
	haz_cube = msg.intake_high;
}

void cube_detection_callback(const cube_detection::CubeDetection &msg)
{
	cube_detection_msg = msg;
}

/*void exchange_detection_callback(const exchange_detection::CubeDetection &msg)
{
	exchange_detection_msg = msg;
}*/
