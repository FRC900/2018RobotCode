#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>


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

	//convert to grayscale and threshold
	cvtColor(frame, gray, CV_BGR2GRAY);
 	threshold(gray, thresh, 50, 255, 0);

	//apply erosion and dilation to filter our noise
	int dilation_size = 5;
	int erosion_size = 5;

        Mat element = getStructuringElement(MORPH_ELLIPSE,
					Size(2*erosion_size + 1, 2*erosion_size+1),
                                        Point(erosion_size, erosion_size));
	erode(thresh, thresh, element);

        element = getStructuringElement(MORPH_ELLIPSE,
                                       Size(2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point(dilation_size, dilation_size));
  	dilate(thresh, thresh, element);

	GaussianBlur(thresh, thresh, Size(61, 61), 0, 0 );

	//extract remaining contours from thresholded image
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	//check if there are contours
	if(contours.size() == 0) {
		//TODO : add ros error message
	} else {
		//iterate through vector and draw contours
		vector<RotatedRect> minRect(contours.size());
		Mat img = Mat::zeros( thresh.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++){
			drawContours(img, contours, i, (0,0,255), 2, 8, hierarchy);
			minRect[i] = minAreaRect(Mat(contours[i]));
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_detect");

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
