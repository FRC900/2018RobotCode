#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
//C
#include <stdio.h>
//C++

#include <sstream>
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

	//convert to grayscale and threshold
	cvtColor(*framePtr, gray, cv::COLOR_BGR2GRAY);
 	threshold(gray, thresh, 50, 255, 0);

	VideoCapture cap(1);
	while(1)
	{
		Mat frame;
		Mat hsv;
		Mat threshold;
		Mat edges;
		Mat contour;
		Mat mask;
		Mat grey;
		Mat out;
		cap >> frame;
		cvtColor(frame,hsv,COLOR_BGR2HSV);
		cvtColor(frame,grey,COLOR_BGR2GRAY);
		inRange(hsv,Scalar(20,89,70),Scalar(35,255,241),threshold);
		Mat threshold_channels[3];
		split(threshold,threshold_channels);
		mask = threshold_channels[0];
		//threshold(hsv,threshold,25,32,THRESH_BINARY);
		erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));
		dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));
		dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));
		erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(4,4)));
		//cvtColor(threshold,threshold,COLOR_HSV2BGR);	
		//Canny(grey,edges,0,30,3);
		//cvtColor(threshold,threshold,COLOR_BGR2GRAY);	
		
		
		//edges.copyTo(out,mask); //There needs to be a line which converts the hsv image into greyscale, then the contours need to be found
		vector<vector<Point> > contours;
		vector<Vec4i> rank;
		//RNG rng(12345);
		findContours(threshold,contours, rank, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
		
		vector<vector<Point> > contours_poly( contours.size() );
		vector<Rect> boundRect( contours.size() );
		
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
		//imshow("edges",edges);
		//imshow("mask",mask);
		//imshow("edges",edges);
		//imshow("out",out);
		imshow("drawing",drawing);
		//imshow("contours",contour);
		if((waitKey(30)) == 27 ) break;
	}
	//return 0;
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

