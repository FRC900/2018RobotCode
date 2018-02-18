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

int hLo = 23;
int sLo = 65;
int vLo = 240;
int hUp = 40;

//double minArea = 86500;
//68860x^2-232300x+202600

//double maxArea = 111560;
//82320x^2-273400x+247600

static bool down_sample = false;
//This funtion along with the commented out slider code is useful when getting new HSV values for the threshold
//To get the trackbars active, comment out the lines marked "mark1", uncomment the lines marked "mark2",
//multiline comment from "markS" to "markE"
void on_trackbar(int, void*)
{}
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

	cvtColor(*framePtr,hsv,COLOR_BGR2HSV);
	inRange(hsv, Scalar(hLo,sLo,vLo),Scalar(hUp,255,255),threshold); //mark2
	//inRange(hsv, Scalar(23,65,240),Scalar(40,255,255),threshold);
	/*Mat threshold_channels[3];
	split(threshold,threshold_channels);
	mask = threshold_channels[0];*/
	//markS
	erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(7,7)));
	dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(7,7)));

	dilate(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
	erode(threshold,threshold,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

	vector<vector<Point> > contours;
	vector<Vec4i> rank;

	findContours(threshold, contours, rank, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<float> contourDepth;


	for(int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]),contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		
	}

	for(int i = 0; i < contours.size(); i++)
	{
		const int x = boundRect[i].x;
		const int y = boundRect[i].y;
		float depth_value = depthPtr->at<float>(y,x);
		contourDepth.push_back(depth_value);
	}
	

	Mat drawing = Mat::zeros(threshold.size(),CV_8UC3);
	for(int i = 0; i< contours.size(); i++)
	{
		double minArea = 68860 * (pow(contourDepth[i], 2)) - (232300 * contourDepth[i]) + 202600;
		double maxArea = 82320 * (pow(contourDepth[i], 2)) - (273400 * contourDepth[i]) + 247600;
		double areaContour = boundRect[i].height * boundRect[i].width;
		Scalar rect_color = Scalar(0,0,255);
		Scalar color = Scalar(0,255,0);		
		drawContours(drawing, contours,i,color,2,8,rank,0,Point());

		if (false/*areaContour < minArea || areaContour > maxArea*/){
			continue;
		} else if (false/*abs((boundRect[i].height/boundRect[i].width) - 1) > 2.0*/) {
			continue;
		} else if (false/*abs((boundRect[i].width/boundRect[i].height) - 1) > 2.0*/) {
			continue;
		} else {	
			rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), rect_color, 2, 8, 0);
		}
	}
	//markE
	imshow("threshold",threshold); //mark1
	imshow("hsv",hsv);
	imshow("drawing",drawing); //mark1 
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

	namedWindow("drawing",1);
		
	createTrackbar( "Lower H", "drawing", &hLo, 180);
	createTrackbar( "Lower S", "drawing", &sLo, 255);  
	createTrackbar( "Lower V", "drawing", &vLo, 255);
	createTrackbar( "Higher H", "drawing", &hUp, 180);
	//createTrackbar( "minArea", "drawing", &minArea, 5000);
	//createTrackbar( "maxArea", "drawing", &maxArea, 1000000);

	ros::spin();

	return 0;
}
