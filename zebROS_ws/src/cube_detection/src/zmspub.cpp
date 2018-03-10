#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Header.h>

#include "zedcamerain.hpp"
#include "zedsvoin.hpp"
#include "zmsin.hpp"

#include <sstream>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "zmspub");
	ros::NodeHandle n;
	int pub_rate = 1;

	//you can set the filename in the config file

	char *zms_file;

	if (ros::parameters::get("zms_file", zms_file))
	{
		std::cerr << "The zms_file parameter could not be loaded" << std::endl;
		return -1;
	}

	cap = new ZMSIn(zms_file);

/*
	MediaIn *cap = NULL;
	if (argc == 2)
		cap = new ZMSIn(argv[1]);
	else
		cap = new ZedCameraIn(true);
*/
	if (cap == NULL)
	{
		cerr << "Error creating input" << endl;
		return -1;
	}

	ros::Publisher zms_pub = n.advertise<cube_detection::zmsMsg>("zms_message", 1);

	Mat image;
	Mat depth;
	while (ros::ok() && cap->getFrame(image, depth))
	{
	cube_detection::zmsMsg msg;

	zmsMsg.image = image;
	zmsMsg.depth = depth;

    	zms_pub.publish(msg);
    	ros::spinOnce();
	}

	return 0;
}

