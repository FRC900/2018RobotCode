#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <sys/types.h>
#include <sys/stat.h>

#include "zmsout.hpp"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

static ZMSOut *zmsOut = NULL;
static bool down_sample = false;

void callback(const ImageConstPtr& frameMsg, const ImageConstPtr& depthMsg)
{
	cv_bridge::CvImagePtr cvFrame = cv_bridge::toCvCopy(frameMsg, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImagePtr cvDepth = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

	// Avoid copies by using pointers to RGB and depth info
	// These pointers are either to the original data or to
	// the downsampled data, depending on the down_sample flag
	const Mat *framePtr = &cvFrame->image;
	const Mat *depthPtr = &cvDepth->image;

	// To hold downsampled images, if necessary
	Mat frame;
	Mat depth;

	// Downsample for speed purposes
	if(down_sample)
	{
		pyrDown(*framePtr, frame);
		pyrDown(*depthPtr, depth);

		// And update pointers to use the downsampled
		// versions of the RGB and depth data
		framePtr = &frame;
		depthPtr = &depth;
	}

	zmsOut->saveFrame(*framePtr, *depthPtr);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_detect");

	char name[PATH_MAX];
	int index = -1;
	int rc;
	struct stat statbuf;
	do
	{
		sprintf(name, "/mnt/900_2/cap%d_0.zms", ++index);
		rc = stat(name, &statbuf);
	}
	while (rc == 0);

	sprintf(name, "/mnt/900_2/cap%d.zms", index);
	zmsOut = new ZMSOut(name, 1, 250, true);

	ros::NodeHandle nh("~");
	nh.getParam("down_sample", down_sample);

	// Sync up timestamps to find image and depth
	// data from the same frame
	message_filters::Subscriber<Image> frame_sub(nh, "/zed_goal/left/image_rect_color", 5);
	message_filters::Subscriber<Image> depth_sub(nh, "/zed_goal/depth/depth_registered", 5);

	typedef sync_policies::ExactTime<Image, Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), frame_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();

	delete zmsOut;

	return 0;
}
