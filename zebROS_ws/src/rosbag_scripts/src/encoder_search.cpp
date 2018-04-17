#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <fstream>
#include <talon_state_controller/TalonState.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
	if(argc < 2)
	{
		std::cout << "need a file thanks \n";
		return 1;
	}
	if(argc > 2)
	{
		std::cout << "why so many files ahhh \n";
		return 1;
	}

	std::string bag_name = argv[1];

	rosbag::Bag bag;
	bag.open(bag_name, rosbag::bagmode::Read);

	std::ofstream temp_file;
	temp_file.open ("temp_file.txt");

	std::vector<std::string> topics;
	topics.push_back(std::string("/frcrobot/talon_states"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));
//name: [bl_angle, bl_drive, br_angle, br_drive, fl_angle, fl_drive, fr_angle, fr_drive, intake1, intake2, lift, lift_follower1, lift_follower2, pivot]
	foreach(rosbag::MessageInstance const m, view)
	{
		talon_state_controller::TalonState::ConstPtr s = m.instantiate<talon_state_controller::TalonState>();
		if (s != NULL){
			temp_file << s->header << std::endl;
		    temp_file << "position: " << s->position[1] << std::endl;
		    temp_file << "position: " << s->position[2] << std::endl;
		    temp_file << "position: " << s->position[3] << std::endl;
		    temp_file << "position: " << s->position[4] << std::endl;
		    temp_file << "position: " << s->position[5] << std::endl;
		    temp_file << "position: " << s->position[6] << std::endl;
		    temp_file << "position: " << s->position[7] << std::endl;
		    temp_file << "position: " << s->position[8] << std::endl;
			temp_file << "speed: " << s->speed[1] << std::endl;
			temp_file << "speed: " << s->speed[2] << std::endl;
			temp_file << "speed: " << s->speed[3] << std::endl;
			temp_file << "speed: " << s->speed[4] << std::endl;
			temp_file << "speed: " << s->speed[5] << std::endl;
			temp_file << "speed: " << s->speed[6] << std::endl;
			temp_file << "speed: " << s->speed[7] << std::endl;
			temp_file << "speed: " << s->speed[8] << std::endl;
		}

	}
	bag.close();

	return 0;
}

