#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <fstream>
#include <std_msgs/Float64.h>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
	std::string bag_name = argv[1];

	rosbag::Bag bag;
	bag.open(bag_name, rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("/frcrobot/error_times"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view)
	{
		std_msgs::Float64::ConstPtr s = m.instantiate<std_msgs::Float64>();
		if (s != NULL){
				std::cout << "error at " << s->data << std::endl;
		}
	}
	bag.close();

	return 0;
}

