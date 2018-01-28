#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Header.h>
#include "navx_publisher/stampedUInt64.h"

#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>

#include "goal_detection/GoalDetection.h"

#include <sstream>

#include "GoalDetector.hpp"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

static ros::Publisher pub;
static GoalDetector *gd = NULL;
static const bool batch = true;
static bool down_sample = false;

void callback(const ImageConstPtr &frameMsg, const ImageConstPtr &depthMsg, const navx_publisher::stampedUInt64ConstPtr &navxMsg)
{
	
	cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

	// Avoid copies by using pointers to RGB and depth info
	// These pointers are either to the original data or to
	// the downsampled data, depending on the down_sample flag
	const Mat *framePtr = &cvFrame->image;
	const Mat *depthPtr = &cvDepth->image;

	// To hold downsampled images, if necessary
	Mat frame;
	Mat depth;

	// Downsample for speed purposes
	if (down_sample)
	{
		pyrDown(*framePtr, frame);
		pyrDown(*depthPtr, depth);

		// And update pointers to use the downsampled
		// versions of the RGB and depth data
		framePtr = &frame;
		depthPtr = &depth;
	}

	// Initialize goal detector object the first time
	// through here. Use the size of the frame
	// grabbed from the ZED messages
	if (gd == NULL)
	{
		const float hFov = 105.;
		const Point2f fov(hFov * (M_PI / 180.),
						  hFov * (M_PI / 180.) * ((float)framePtr->rows / framePtr->cols));
		gd = new GoalDetector(fov, framePtr->size(), !batch);
	}
	gd->findBoilers(*framePtr, *depthPtr);
	Mat tempFrame(framePtr->clone());
	gd->drawOnFrame(tempFrame, gd->getContours(tempFrame));

		rectangle(tempFrame, gd->goal_rect(), Scalar(0,0,255), 2);
		imshow ("Image", tempFrame);
		waitKey(5);

	const Point3f pt = gd->goal_pos();

	goal_detection::GoalDetection gd_msg;
	gd_msg.header.seq = frameMsg->header.seq;
	gd_msg.header.stamp = frameMsg->header.stamp;
	gd_msg.header.frame_id = frameMsg->header.frame_id;
	gd_msg.location.x = pt.x;
	gd_msg.location.y = pt.y;
	gd_msg.location.z = pt.z;
	gd_msg.valid = gd->Valid();
	gd_msg.navx_timestamp = navxMsg->data;
	pub.publish(gd_msg);

	if (!batch)
	{
		Mat thisFrame(framePtr->clone());
		gd->drawOnFrame(thisFrame, gd->getContours(cvFrame->image));
		imshow("Image", thisFrame);
		waitKey(5);
	}

	if (gd_msg.valid == false)
	{
		return;
	}
	
	/*
	//Transform between goal frame and odometry/map.
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = cvFrame->header.frame_id;
	transformStamped.child_frame_id = "goal";

	transformStamped.transform.translation.x = gd_msg.location.x;
	transformStamped.transform.translation.y = gd_msg.location.y;
	transformStamped.transform.translation.z = gd_msg.location.z;

	tf2::Quaternion q;
	q.setRPY(0, 0, 0);

	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);

	//Transform between a fixed frame and the goal.
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	geometry_msgs::TransformStamped transformStampedOdomCamera;
	try
	{
		transformStampedOdomCamera = tfBuffer.lookupTransform("odom", cvFrame->header.frame_id,
									 ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		ros::Duration(1.0).sleep();
		return;
	}

	geometry_msgs::TransformStamped transformStampedOdomGoal;

	tf2::doTransform(transformStamped, transformStampedOdomGoal, transformStampedOdomCamera);

	br.sendTransform(transformStampedOdomGoal);
	*/
}

void callbackNavx(const ImageConstPtr &frameMsg, const ImageConstPtr &depthMsg, const navx_publisher::stampedUInt64ConstPtr &navxMsg)
{
	cout << "callback navx" << endl;
	callback(frameMsg, depthMsg, navxMsg);
}

void callbackNoNavx(const ImageConstPtr &frameMsg, const ImageConstPtr &depthMsg)
{
	navx_publisher::stampedUInt64 fakeMsg;
	fakeMsg.header.stamp = ros::Time::now();
	fakeMsg.data = 0;
	const boost::shared_ptr<navx_publisher::stampedUInt64> ptr = boost::make_shared<navx_publisher::stampedUInt64>(fakeMsg);
	cout << "Callback no navx" << endl;
	callback(frameMsg, depthMsg, ptr);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_detect");

	ros::NodeHandle nh("~");
	down_sample = false;
	int sub_rate = 10;
	int pub_rate = 1;
	nh.getParam("down_sample", down_sample);
	nh.getParam("sub_rate", sub_rate);
	nh.getParam("pub_rate", pub_rate);
	message_filters::Subscriber<Image> frame_sub(nh, "/zed_goal/left/image_rect_color", sub_rate);
	message_filters::Subscriber<Image> depth_sub(nh, "/zed_goal/depth/depth_registered", sub_rate);
	message_filters::Subscriber<navx_publisher::stampedUInt64> navx_sub(nh, "/navx/time", 50);

	ros::Duration wait_t(5.0); //wait 5 seconds for a navx publisher
	ros::Time stop_t = ros::Time::now() + wait_t;
	while (ros::Time::now() < stop_t &&navx_sub.getSubscriber().getNumPublishers() == 0)
	{
		ros::Duration(0.5).sleep();
		ros::spinOnce();
		ROS_INFO_STREAM("Waiting for a navx publisher" << endl);
	}

	typedef sync_policies::ApproximateTime<Image, Image > MySyncPolicy2;
	typedef sync_policies::ApproximateTime<Image, Image, navx_publisher::stampedUInt64> MySyncPolicy3;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(xxx)
	Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(50), frame_sub, depth_sub);
	Synchronizer<MySyncPolicy3> sync3(MySyncPolicy3(50), frame_sub, depth_sub, navx_sub);
	if (navx_sub.getSubscriber().getNumPublishers() == 0)
	{
		ROS_WARN_STREAM("Navx not found, running in debug mode" << endl);
		sync2.registerCallback(boost::bind(&callbackNoNavx, _1, _2));
	}
	else
	{
		sync3.registerCallback(boost::bind(&callbackNavx, _1, _2, _3));
	}

	// Set up publisher
	pub = nh.advertise<goal_detection::GoalDetection>("goal_detect_msg", pub_rate);

	ros::spin();

	delete gd;


	return 0;
}
