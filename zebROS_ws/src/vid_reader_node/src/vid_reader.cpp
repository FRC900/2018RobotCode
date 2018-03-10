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
	
	ros::NodeHandle nh("~");
	int sub_rate = 5;
	int pub_rate = 1;

	ros::Publisher zms_pub;
	ros::Publisher zms_pub1;

	MediaIn *cap = NULL;
	String video_file = "cap6_0.zms";
		cap = new ZMSIn(argv[1]);



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
		
		sensor_msgs::Image depth_in;
		sensor_msgs::Image rgb_in;

		vid_reader_msg.header.seq = 1;
		vid_reader_msg.header.stamp = ros::Time::now();
		vid_reader_msg.header.frame_id;

		rgb_in.height = image.rows;
		rgb_in.width = image.cols;

		depth_in.height = depth.rows;
		depth_in.width = depth.cols;

		//cv_bridge::CvImage img_bridge;
		//std_msgs::Header header;
		//header.seq = vid_reader_msg.header.seq;
		//header.stamp = ros::Time::now();
 

		//img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, rgb_in);
		//img_bridge.toImageMsg(rgb_in);

		
		

		cv_bridge::CvImagePtr cv_ptr;
	        cv_ptr = cv_bridge::toCvCopy(rgb_in, sensor_msgs::image_encodings::BGR8);
		zms_pub.publish(rgb_in);

		cv_bridge::CvImagePtr cv_ptr1;
		cv_ptr1 = cv_bridge::toCvCopy(depth_in, sensor_msgs::image_encodings::BGR8);
		zms_pub1.publish(depth_in);

		stringstream ss;
		ss << fixed << setprecision(2) << cap->FPS() << "C:" << frameTicker.getFPS() << "GD FPS";
		putText(image, ss.str(), Point(image.cols - 15 * ss.str().length(), 50), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255));
		
		// /zed/rgb/image_rect_color
		// /zed/depth/depth_registered
		zms_pub = nh.advertise<sensor_msgs::Image>("vid_reader_rgb_msg", pub_rate);
		zms_pub1 = nh.advertise<sensor_msgs::Image>("vid_reader_depth_msg", pub_rate);
		
		imshow ("Image", image);

		if ((uchar)waitKey(5) == 27)
			break;
	}
	return 0;
}
