#include "auto_preseason/auto_loop.h"

static ros::Publisher cmd_vel_pub;
static ros::Subscriber cube_state_sub;
static ros::Subscriber cube_detection_sub;
static ros::Subscriber qr_code_sub;
static ros::ServiceClient path_to_cube_srv;
static ros::ServiceClient turn_to_angle_srv;
static ros::ServiceClient intake_srv;

std::shared_ptr<actionlib::SimpleActionClient<path_to_cube::PathAction>> ac_cube;
std::shared_ptr<actionlib::SimpleActionClient<zbar_ros::PathAction>> ac_exchange;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> ac_intake;

/*
 * TODO
 * make pathing actionlib into "go to a place" instaed of cube vs exchange
 * what needs to be services and what needs to be actionlib?
 *  - pathing to exchange/cube
 *  - turning to an angle
 *  - intaking/vaulting cubes
 */

//Turn slowly in a circle
bool angle_to_exchange()//different for echange and cubes, include navX angle 
{
	ROS_INFO_NAMED("auto loop", "recovery_mode_exchange");
	auto_preseason::Angle srv;
	srv.request.angle = 0; //or whatever
	turn_to_angle_srv.call(srv);

	return true;	
}

bool angle_to_cube()//different for echange and cubes, include navX angle 
{
	ROS_INFO_NAMED("auto loop", "recovery_mode_cube");
	auto_preseason::Angle srv;
	srv.request.angle = 180; //or whatever
	turn_to_angle_srv.call(srv);

	return true;	
}

bool recovery_mode()
{
	ROS_INFO_NAMED("auto loop", "recovery_mode_default");
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

bool start_path_to_cube()
{
	ROS_INFO_NAMED("auto loop", "path_to_cube");
	path_to_cube::PathGoal goal;
	ac_cube->sendGoal(goal);
	return true; //these should maybe just be voids
}

bool start_intake_cube()
{
	ROS_INFO_NAMED("auto loop", "intake_cube");

	//fix the angle
	auto_preseason::Angle angle; 
	angle.request.angle = cube_detection_msg.angle; //make sure this is from the same ref. point and that data is updated!
	if (!turn_to_angle_srv.call(angle))
	{
		ROS_ERROR("Turn to angle service call failed in auto_loop");
		//return false; //comment out for simulation
	}

	//start the actionlib goal
	behaviors::IntakeGoal goal;

    goal.IntakeCube = true;
    goal.time_out = 10; //TODO config this
    goal.wait_to_proceed = false; 

    ac_intake->sendGoal(goal);
	

	haz_cube = true; //for simulation

	return true;
}

bool drive_forward_slowly()
{
	geometry_msgs::Twist vel;
	vel.linear.x = 0.1;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	vel.angular.z = 0.0;
	cmd_vel_pub.publish(vel);

	return true;
}

bool start_path_to_exchange()
{
	ROS_INFO_NAMED("auto loop", "path_to_exchange");
	/*path_to_exchange::PathGoal goal; //make a dummy one?
	ac_exchange.sendGoal(goal);*/
	return true;
}

bool start_vault_cube()
{
	ROS_INFO_NAMED("auto loop", "start_vault_cube");

	behaviors::IntakeGoal goal;

    goal.IntakeCube = false;
    goal.time_out = 10; //TODO config this
    goal.wait_to_proceed = false; 

    ac_intake->sendGoal(goal);
	haz_cube = false;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_loop");
	ros::NodeHandle n;

	ac_cube = std::make_shared<actionlib::SimpleActionClient<path_to_cube::PathAction>> ("path_cube", true);
	ac_exchange = std::make_shared<actionlib::SimpleActionClient<zbar_ros::PathAction>> ("path_exchange", true);
	ac_intake = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>> ("intake", true);

	ROS_INFO_STREAM("waiting for server to start");
	//ac_cube->waitForServer();
	//ac_exchange->waitForServer();
	//ac_intake->waitForServer();
	ROS_INFO_STREAM("action server started");

	enum State {STARTING_STATE,
		NO_CUBE_SEEN,
		RECOVERY_MODE_CUBE,
		CUBE_SEEN_FAR,
		PATHING_TO_CUBE,
		CUBE_SEEN_CLOSE,
		INTAKING_CUBE,
		NO_EXCHANGE_SEEN,
		RECOVERY_MODE_EXCHANGE,
		EXCHANGE_SEEN_FAR,
		PATHING_TO_EXCHANGE,
		EXCHANGE_SEEN_CLOSE,
		VAULTING_CUBE
	};

	State state = STARTING_STATE;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/frcrobot/swerve_drive_controller/cmd_vel", 1);
	cube_state_sub = n.subscribe("/frcrobot/elevator_controller/cube_state", 1, &cube_state_callback);
	cube_detection_sub = n.subscribe("/frcrobot/cube_detection/cube_detection_something", 1, &cube_detection_callback);
	qr_code_sub = n.subscribe("/barcode", 1, &exchange_detection_callback);
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
		if(cube_found)
			cube_dist = cube_detection_msg.location[0].z;
		exchange_dist = 5.0; //figure out which sensor this will be

		switch(state)
		{
			case STARTING_STATE : 
							{
								ROS_INFO_STREAM("starting state");
								if(haz_cube)
								  {
									  if(qr_found)
									  {
										  if(exchange_dist < 1) //TODO: figure out which sensor is going to be used for distance to wall (ultrasonic)
											  state = EXCHANGE_SEEN_CLOSE;
										  else
											  state = EXCHANGE_SEEN_FAR;
									  }
									  else
										  state = NO_EXCHANGE_SEEN;
								  }
								  else if(!haz_cube && cube_dist >= min_pathing_dist && cube_found)
									  state = CUBE_SEEN_FAR;
								  else if(!haz_cube && cube_dist <= min_pathing_dist  && cube_found)
									  state = CUBE_SEEN_CLOSE;
								  else if(!cube_found)
									  state = NO_CUBE_SEEN;
							}
			case NO_CUBE_SEEN :
							{
								  ROS_INFO_STREAM("no cube seen");
								  angle_to_cube();
								  if(cube_found)
								  {
									  if(cube_dist < 5)
										  state = CUBE_SEEN_CLOSE;
									  if(cube_dist >= 5)
										  state = CUBE_SEEN_FAR;
								  }
								  else
									  state = RECOVERY_MODE_CUBE;
							}
			case RECOVERY_MODE_CUBE : 
							{
								  ROS_INFO_STREAM("recovery mode cube");
								  recovery_mode();
								  cube_found = true; //for simulation
								  if(cube_found)
								  {
									  if(cube_dist < 5)
										  state = CUBE_SEEN_CLOSE;
									  if(cube_dist >= 5)
										  state = CUBE_SEEN_FAR;
								  }
							}
			case CUBE_SEEN_FAR : 
							{
								  ROS_INFO_STREAM("cube seen far");
								  start_path_to_cube(); //TODO: needs to be an actionlib thing -- can start and then run in the background
								  state = PATHING_TO_CUBE;
							}
			case PATHING_TO_CUBE : 
							{
								  ROS_INFO_STREAM("pathing to cube");
								  actionlib::SimpleClientGoalState path_state = ac_cube->getState();
								  if(path_state.toString() == "SUCCEEDED") //TODO: check pathing sending back actionlib stuff
								  {
									  //wait a certain amount of time before checking for a cube?
									  state = CUBE_SEEN_CLOSE;
								  }
								  else
									  ROS_INFO_STREAM("path_to_cube has failed. state: " << path_state.toString());
							}
			case CUBE_SEEN_CLOSE :
							{
								  ROS_INFO_STREAM("cube seen close");
								  start_intake_cube();
								  state = INTAKING_CUBE;
							}
			case INTAKING_CUBE :
							{
								  drive_forward_slowly();
								  ROS_INFO_STREAM("intaking cube");
								  actionlib::SimpleClientGoalState intake_state = ac_intake->getState();
								  if(intake_state.toString() == "SUCCEEDED")
								  {
									  if(qr_found)
									  {
										  if(exchange_dist < 1)
											  state = EXCHANGE_SEEN_CLOSE;
										  else
											  state = EXCHANGE_SEEN_FAR;
									  }
									  else
										  state = NO_EXCHANGE_SEEN;
								  }
							}
			case NO_EXCHANGE_SEEN :
							{
								  ROS_INFO_STREAM("no exchange seen");
								  angle_to_exchange();
								  if(qr_found)
								  {
									  if(exchange_dist < 1)
										  state = EXCHANGE_SEEN_CLOSE;
									  else
										  state = EXCHANGE_SEEN_FAR;
								  }
								  else
									  state = RECOVERY_MODE_EXCHANGE;
							}
			case RECOVERY_MODE_EXCHANGE :
							{
								  ROS_INFO_STREAM("recovery_mode_exchange");
								  recovery_mode();
								  qr_found = true; //for simulation
								  if(qr_found)
								  {
									  if(exchange_dist < 1)
										  state = EXCHANGE_SEEN_CLOSE;
									  else
										  state = EXCHANGE_SEEN_FAR;
								  }
							}
			case EXCHANGE_SEEN_FAR : 
							{
								  ROS_INFO_STREAM("exchange seen far");
								  start_path_to_exchange(); //TODO: needs to be an actionlib thing -- can start and then run in the background
								  state = PATHING_TO_EXCHANGE;
							}
			case PATHING_TO_EXCHANGE :
							{
								  ROS_INFO_STREAM("pathing to exchange");
								  actionlib::SimpleClientGoalState exchange_state = ac_exchange->getState();
								  if(exchange_state.toString() == "SUCCEEDED") //TODO: check pathing sending back actionlib stuff
								  {
									  //wait until you see the exchange?
									  state = EXCHANGE_SEEN_CLOSE;
								  }
								  else
									  ROS_INFO_STREAM("path_to_cube has failed. state: " << exchange_state.toString());
							}
			case EXCHANGE_SEEN_CLOSE :
							{
								  ROS_INFO_STREAM("exchange seen close");
								  start_vault_cube(); //TODO: some actionlib stuff
								  state = VAULTING_CUBE;
								  ROS_INFO_STREAM("vaulting cube");
							}
			case VAULTING_CUBE :
							{
								  actionlib::SimpleClientGoalState intake_state = ac_intake->getState();
								  if(intake_state.toString() == "SUCCEEDED") //TODO: some actionlib stuff
								  {
									  if(true) //TODO: figure out what sensor needs to determine this
										  state = NO_CUBE_SEEN; //has to ignore the cube that it's looking at
									  if(false)
										  state = CUBE_SEEN_CLOSE;
								  }
							}
		}
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

void exchange_detection_callback(const cube_detection::CubeDetection &msg)
{
	exchange_detection_msg = msg;
}
