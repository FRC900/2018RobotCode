#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"

#include <math.h>

void modes(const std_msgs::StringConstPtr& mode) {
    return;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "Autonomous mode subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("autonomous_mode", 1, modes);

    ros::spin();
    return 0;
}
