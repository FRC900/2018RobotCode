#pragma once
// Use ZED SVO input files. Similar to other video in classes.
// Runs an update thread in the background which tries to
// decode one frame ahead of the frame used in getFrame().

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#ifdef ZED_SUPPORT
#include <sl/Camera.hpp>
#endif
#include "syncin.hpp"
#include "zedparams.hpp"

class ZedSVOIn : public SyncIn
{
	public:
		ZedSVOIn(const char *inFileName = NULL, ZvSettings *settings = NULL);
		~ZedSVOIn();

#ifdef ZED_SUPPORT
		// Make class non-copyable
		ZedSVOIn(const ZedSVOIn &sedsvoin) = delete;
		ZedSVOIn& operator=(const ZedSVOIn &sedsvoin) = delete;

		bool         isOpened(void) const;

		// How many frames?
		int          frameCount(void) const;
		CameraParams getCameraParams(void) const;
#endif
	protected:
		// Defined in derived classes to handle the nuts
		// and bolts of grabbing a frame from a given
		// source.  preLock happens before the mutex
		// while postLock happens inside it
		bool postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud);
		bool postLockFrameNumber(int framenumber);
#ifdef ZED_SUPPORT
	private:
		sl::Camera                        zed_;
		ZedParams                         params_;
		sl::Mat                           localFrameZed_;
		cv::Mat                           localFrame_;
		sl::Mat                           localDepthZed_;
		cv::Mat                           localDepth_;
		sl::Mat                           localCloudZed_;
        pcl::PointCloud<pcl::PointXYZRGB> localCloud_;
		bool                              opened_;
		int                               frames_;

#endif
};
