#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "videoin.hpp"
#include "ZvSettings.hpp"

using namespace cv;

VideoIn::VideoIn(const char *inpath, ZvSettings *settings) :
	SyncIn(settings),
	cap_(inpath)
{
	if (cap_.isOpened())
	{
		width_  = cap_.get(CV_CAP_PROP_FRAME_WIDTH);
		height_ = cap_.get(CV_CAP_PROP_FRAME_HEIGHT);

		// getNextFrame scales down large inputs
		// make width and height match adjusted frame size
		while (height_ > 800)
		{
			width_ /= 2;
			height_ /= 2;
		}
		frames_ = cap_.get(CV_CAP_PROP_FRAME_COUNT);
		startThread();
	}
	else
		std::cerr << "Could not open input video "<< inpath << std::endl;
}


VideoIn::~VideoIn()
{
	stopThread();
}


bool VideoIn::isOpened(void) const
{
	return cap_.isOpened();
}


bool VideoIn::postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	cap_ >> frame;
	depth = Mat();
	cloud.clear();
	return true;
}


int VideoIn::frameCount(void) const
{
	return frames_;
}


bool VideoIn::postLockFrameNumber(int framenumber) 
{
	if (framenumber < frames_)
	{
		cap_.set(CV_CAP_PROP_POS_FRAMES, framenumber);
		return true;
	}
	return false;
}
