#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "zmq.hpp"

using namespace std;

class C
{
	private:
		zmq::context_t context;
		zmq::socket_t publisher;
	public:
		C(void):
			context(1),
			publisher(context, ZMQ_PUB)
		{
			publisher.bind("tcp://*:5800");
		}

		void callback(const geometry_msgs::Twist& vel) 
		{
			stringstream ss;
			ss << "C ";
			ss << fixed << setprecision(6) << vel.linear.x << " ";
			ss << fixed << setprecision(6) << vel.linear.y << " ";
			ss << fixed << setprecision(6) << vel.angular.z;

			cout << ss.str() << endl;
			zmq::message_t grequest(ss.str().length() - 1);
			memcpy((void *)grequest.data(), ss.str().c_str(), ss.str().length() - 1);
			publisher.send(grequest);
		}
};

int main(int argc, char **argv)
{
	C c;
	ros::init(argc, argv, "cmd_vel_to_zmq");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/cmd_vel", 2, &C::callback, &c);

	ros::spin();
	return 0;
}
