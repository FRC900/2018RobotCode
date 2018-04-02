#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <fstream>
#include <ros_control_boilerplate/MatchSpecificData.h>
#include <pdp_state_controller/PDPData.h>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
	std::string bag_name = argv[1];

	std::string c_string = argv[2];
	int current_needed = boost::lexical_cast<int>(c_string);
	std::cout << current_needed;

	rosbag::Bag bag;
	bag.open(bag_name, rosbag::bagmode::Read);

	std::ofstream temp_file;
	temp_file.open ("temp_file.txt");

	std::ofstream times;
	times.open ("times.txt");

	std::vector<std::string> topics;
	topics.push_back(std::string("/frcrobot/pdp_states"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	pdp_state_controller::PDPData::ConstPtr s = view[0].instantiate<pdp_state_controller::PDPData>();
	if (s != NULL) 
		int start_time = floor(s->header.stamp.toSec());

	foreach(rosbag::MessageInstance const m, view)
	{
		pdp_state_controller::PDPData::ConstPtr s = m.instantiate<pdp_state_controller::PDPData>();
		if (s != NULL){
			temp_file << "current at " << current_needed << ": " <<  s->current[current_needed] << std::endl;
			if  (s->current[current_needed] > 0)
					times << "current at channel " << current_needed << " is significant at header " << floor(s->header.stamp.toSec()) - start_time << std::endl;
		}

	}
	bag.close();

	return 0;
}

