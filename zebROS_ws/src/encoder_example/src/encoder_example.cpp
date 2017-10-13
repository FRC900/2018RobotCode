#include <iostream>

#include <ros/ros.h>

#include "encoder_example/canTalonEncoder.h"
#include "ctrlib/CANTalon.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "encoder_example");

	ros::NodeHandle nh;

	// Set up publishers - data is a standard ROS header (timestamp, frame, seq_number)
	// plus the encoder values
	ros::Publisher pub = nh.advertise<encoder_example::canTalonEncoder>("/encoder", 50);

	encoder_example::canTalonEncoder stampedEncoderData;
	stampedEncoderData.header.frame_id = "encoder_frame"; // make me a parameter?

	ros::Rate loop_time(50); // run at 50Hz

	CANTalon canTalon(11); // make id number a param instead of hard-coded

	while(ros::ok()) 
	{
		//set the timestamp for all headers
		stampedEncoderData.header.stamp = ros::Time::now();
		stampedEncoderData.position = canTalon.GetEncPosition();
		stampedEncoderData.velocity = canTalon.GetEncVel();
		// Need to decide exactly what to publish.  For this simple
		// example it is just a raw encoder values. For a swerve drive
		// wheel we might need velocity in X and Y directions.  For
		// a shooter we might just report velocity.  For linear actuators
		// maybe publish a 1-d position
		//
		// Or do we have other topics which read the raw encoder values
		// and convert them into real-world units of position / velocity?
		//
		//publish to ROS topics
		pub.publish(stampedEncoderData); // queue up data to publish

		ros::spinOnce();        // yield and publish data
		loop_time.sleep();      // sleep until next update period hits
	}

	return 0;
}
