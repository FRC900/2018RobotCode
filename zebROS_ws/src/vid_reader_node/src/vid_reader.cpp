#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "zedcamerain.hpp"
#include "zedsvoin.hpp"
#include "zmsin.hpp"
#include "frameticker.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "vid_reader_node/VidReader.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vid_reader");
	ros::NodeHandle nh("~");
	int sub_rate = 5;
	int pub_rate = 1;
	nh.getParam("pub_rate", pub_rate);

	ros::Publisher zms_pub;
	ros::Publisher zms_pub1;

	MediaIn *cap = NULL;

	string file_path;
	if (!nh.getParam("file_path", file_path))
	{
		ROS_ERROR("You need to pass a file_path argument of the .zms file");
	}
	const char* video_file = file_path.c_str();

	cap = new ZMSIn(video_file);

	if (cap == NULL)
	{
		cerr << "Error creating input" << endl;
		return -1;
	}

	Mat image;
	Mat depth;
	Rect bound;
	String *zed;
	FrameTicker frameTicker;
	while (cap->getFrame(image, depth))
	{
		frameTicker.mark();

		vid_reader_node::VidReader vid_reader_msg;
		
		//ROS_INFO_STREAM("Depth: " << depth << endl);

		vid_reader_msg.header.seq = 1;
		vid_reader_msg.header.stamp = ros::Time::now();
		vid_reader_msg.header.frame_id;

		cv_bridge::CvImage rgb_out;

		try {
	        	rgb_out.encoding = sensor_msgs::image_encodings::BGR8;
			rgb_out.image = image;
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		cv_bridge::CvImage depth_out;

		try {
			depth_out.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			depth_out.image = depth;
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		stringstream ss;
		ss << fixed << setprecision(2) << cap->FPS() << "C:" << frameTicker.getFPS() << "GD FPS";
		putText(image, ss.str(), Point(image.cols - 15 * ss.str().length(), 50), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255));
		ros::Duration(.1).sleep();

		zms_pub = nh.advertise<sensor_msgs::Image>("/zed_goal/left/image_rect_color", pub_rate);
		zms_pub1 = nh.advertise<sensor_msgs::Image>("/zed_goal/depth/depth_registered", pub_rate);

		zms_pub.publish(rgb_out.toImageMsg());
		zms_pub1.publish(depth_out.toImageMsg());

		//imshow ("Image", image);
		//imshow ("Depth", depth);

		if ((uchar)waitKey(5) == 27)
			break;
	}
	return 0;
}
