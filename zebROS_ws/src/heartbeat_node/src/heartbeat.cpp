#include <std_msgs/Int32.h>
#include <ros/ros.h>

ros::Publisher heartbeat_pub_rio;

int main(int argc, char** argv) {
    ros::init(argc, argv, "heartbeat_rio");
    ros::NodeHandle n;
    
    heartbeat_pub_rio = n.advertise<std_msgs::Int32>("heartbeat", 1);

    ros::Rate r(5);
    std_msgs::Int32 val;
    val.data = 0;
    int num = 1;
    while(ros::ok()){
       val.data = num;
       num++;
       heartbeat_pub_rio.publish(val);
       r.sleep();
    }
}
