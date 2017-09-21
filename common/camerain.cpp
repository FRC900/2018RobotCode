#include <iostream>
#include <opencv2/opencv.hpp>

#include "camerain.hpp"

using namespace cv;
using namespace std;

CameraIn::CameraIn(int stream, ZvSettings *settings) :
	AsyncIn(settings),
	fps_(30.),
	cap_(stream)
{
	if (cap_.isOpened())
	{
		// Defaults, might be overridden by saved 
		// setting
		realWidth_  = width_ = 1280;
		realHeight_ = height_ = 720;

		if (!loadSettings()) {
			cerr << "Failed to load CameraIn settings" << endl;
		}

		cap_.set(CV_CAP_PROP_FPS, fps_);
		cap_.set(CV_CAP_PROP_FRAME_WIDTH, width_);
		cap_.set(CV_CAP_PROP_FRAME_HEIGHT, height_);

		// getNextFrame resizes large inputs,
		// make sure width and height match
		while (height_ > 700)
		{
			width_ /= 2;
			height_ /= 2;
		}
		startThread();
	}
	else
		std::cerr << "Could not open camera" << std::endl;
}

CameraIn::~CameraIn()
{
	stopThread();
	saveSettings();
}

bool CameraIn::loadSettings(void)
{
	if (settings_) {
		settings_->getDouble(getClassName(), "fps", fps_);
		settings_->getUnsignedInt(getClassName(), "width", width_);
		settings_->getUnsignedInt(getClassName(), "height", height_);
		realWidth_ = width_;
		realHeight_ = height_;
		return true;
	}
	return false;
}

bool CameraIn::saveSettings(void) const
{
	if (settings_) {
		settings_->setDouble(getClassName(), "fps", fps_);
		settings_->setInt(getClassName(), "width", realWidth_);
		settings_->setInt(getClassName(), "height", realHeight_);
		settings_->save();
		return true;
	}
	return false;
}

bool CameraIn::isOpened() const
{
	return cap_.isOpened();
}

// Grab & retrieve frame from camera into local
// buffer. Since this doesn't update shared vars
// it can be before the lock is grabbed
bool CameraIn::preLockUpdate(void)
{
	return cap_.grab() && cap_.retrieve(localFrame_);
}

bool CameraIn::postLockUpdate(Mat &frame, Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	localFrame_.copyTo(frame);
	depth = Mat();
	cloud.clear();
	return true;
}

