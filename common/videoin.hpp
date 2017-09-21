// Input class to handle RGB (or grayscale?) video inputs :
// MPG, AVI, MP4, etc.
// Code runs a separate decode thread which tries to buffer
// one frame ahead of the data needed by getFrame
#pragma once

#include <opencv2/core/core.hpp>
#include "syncin.hpp"

class ZvSettings;

class VideoIn : public SyncIn
{
	public:
		VideoIn(const char *inpath, ZvSettings *settings = NULL);
		~VideoIn();

		bool isOpened(void) const;

		int  frameCount(void) const;

	protected:
		// Defined in derived classes to handle the nuts
		// and bolts of grabbing a frame from a given
		// source.  preLock happens before the mutex
		// while postLock happens inside it
		bool postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud);
		bool postLockFrameNumber(int framenumber);

	private:
		cv::VideoCapture cap_;
		int              frames_;

		bool loadSettings(void) { return true; }
		bool saveSettings(void) const { return true; }
		std::string getClassName() const { return "VideoIn"; }
};
