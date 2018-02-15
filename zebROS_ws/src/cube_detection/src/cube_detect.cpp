#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

int erosion_size = 1;
const int sliderMax = 255;

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

static bool down_sample = false;

void callback(const ImageConstPtr &frameMsg, const ImageConstPtr &depthMsg)
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

 	Mat gray;
	Mat thresh;

	// Downsample for speed purposes
	if (down_sample)
	{
		pyrDown(*framePtr, frame);
		pyrDown(*depthPtr, depth);

		framePtr = &frame;
		depthPtr = &depth;
	}

	Mat hsv;
	Mat threshold;
	Mat contour;

	inRange(hsv, Scalar(20,89,70),Scalar(35,255,241),threshold);
	/*Mat threshold_channels[3];
	split(threshold,threshold_channels);
	mask = threshold_channels[0];*/

	erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));
	dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));
	dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));
	erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));

	vector<vector<Point> > contours;
	vector<Vec4i> rank;

	findContours(threshold,contours, rank, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect(contours.size());

	for(int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]),contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
	}
	Mat drawing = Mat::zeros(threshold.size(),CV_8UC3);
	for(int i = 0; i<contours.size(); i++)
	{
		Scalar color = Scalar(0,255,0);
		drawContours(drawing, contours,i,color,2,8,rank,0,Point());
		rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
	}

	imshow("threshold",threshold);
	imshow("hsv",hsv);
	imshow("drawing",drawing);
	waitKey(5);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cube_detect");

	ros::NodeHandle nh("~");
	int sub_rate = 5;
	message_filters::Subscriber<Image> frame_sub(nh, "/zed_goal/left/image_rect_color", sub_rate);
	message_filters::Subscriber<Image> depth_sub(nh, "/zed_goal/depth/depth_registered", sub_rate);

	typedef sync_policies::ApproximateTime<Image, Image > MySyncPolicy2;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(xxx)
	Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(50), frame_sub, depth_sub);
	sync2.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();

	return 0;
}
