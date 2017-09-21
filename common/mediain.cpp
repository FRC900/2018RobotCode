#include <string>
#include <sys/time.h>

#include "mediain.hpp"
#include "ZvSettings.hpp"

using namespace std;
using namespace cv;


MediaIn::MediaIn(ZvSettings *settings) :
	width_(0),
	height_(0),
	settings_(settings),
	usePointCloud_(false),
	frameNumber_(0),
	lockedFrameNumber_(0),
	timeStamp_(0), // TODO : default to setTimeStamp() instead?
	lockedTimeStamp_(0),
	NavXHandle("/dev/ttyACM0") // TODO : autodetect port NavX is on
{
}


bool MediaIn::loadSettings()
{
	// MediaIn has no settings to load currently
	return true;
}


bool MediaIn::saveSettings() const
{
	// MediaIn has no settings to save currently
	return true;
}


unsigned int MediaIn::width(void) const
{
   return width_;
}


unsigned int MediaIn::height(void) const
{
   return height_;
}

bool MediaIn::getFrame(Mat &frame, bool pause)
{
	(void)pause;
	frame = Mat();
	return false;
}

bool MediaIn::getFrame(Mat &frame, Mat &depth, bool pause)
{
	depth = Mat();
	// Call derived class version of non-depth method
	// to get frame data, if any
	return getFrame(frame, pause);
}

bool MediaIn::getFrame(Mat &frame, Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause)
{
	usePointCloud_ = true;
	cloud = pcl::PointCloud<pcl::PointXYZRGB>();
	// Call derived class version of non-point cloud method
	// to get frame+depth data, if any
	return getFrame(frame, depth, pause);
}

bool MediaIn::getDepth(Mat &depth, bool pause)
{
	(void)pause;
	depth = Mat();
	return false;
}

bool MediaIn::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause)
{
	(void)pause;
	usePointCloud_ = true;
	cloud = pcl::PointCloud<pcl::PointXYZRGB>();
	return false;
}
bool MediaIn::isOpened(void) const
{
	return false;
}

int MediaIn::frameCount(void) const
{
	return -1;
}

// set the time stamp to match the frame just read
// during update().  Since update can run in a 
// separate thread after getFrame() was called, don't
// use this variable for anything returned outside of 
// the class. Intead keep a separate locked version which
// gets updated when getFrame is called. That way the
// timestamp returned outside the class always matches
// up with the frame returned from getFrame()
void MediaIn::setTimeStamp(long long timeStamp)
{
	if (timeStamp != -1)
	{
		timeStamp_ = timeStamp;
	}
	else if(NavXHandle.IsConnected())
	{
		timeStamp_ = NavXHandle.GetLastSensorTimestamp();
	}
	else
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);

		timeStamp_ = (long long)tv.tv_sec * 1000000ULL +
					 (long long)tv.tv_usec;
	}
}

void MediaIn::lockTimeStamp(void)
{
	lockedTimeStamp_ = timeStamp_;
}

long long MediaIn::timeStamp(void) const
{
	return lockedTimeStamp_;
}

// Same thing with frame numbers
void MediaIn::setFrameNumber(int frameNumber)
{
	frameNumber_ = frameNumber;
}

void MediaIn::incFrameNumber(void)
{
	frameNumber_ += 1;
}

void MediaIn::lockFrameNumber(void)
{
	lockedFrameNumber_ = frameNumber_;
}

int MediaIn::frameNumber(void) const
{
	return lockedFrameNumber_;
}

void MediaIn::frameNumber(int frameNumber)
{
	setFrameNumber(frameNumber);
	lockFrameNumber();
}

void MediaIn::FPSmark(void)
{
	frameTicker.mark();
}

float MediaIn::FPS(void) const
{
	return frameTicker.getFPS();
}

CameraParams MediaIn::getCameraParams(void) const
{
	return CameraParams();
}
