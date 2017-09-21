#pragma once

//opencv include
#include <opencv2/core/core.hpp>
#include "syncin.hpp"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>

#include "portable_binary_iarchive.hpp"

class ZMSIn : public SyncIn
{
	public:
		ZMSIn(const char *inFileName = NULL, ZvSettings *settings = NULL);
		~ZMSIn();

		// Make class non-copyable
		ZMSIn(const ZMSIn& zmsin) = delete;
		ZMSIn& operator=(const ZMSIn& zmsin) = delete;

		// How many frames?
		int frameCount(void) const;

		bool isOpened(void) const;

		CameraParams getCameraParams(void) const;

	protected:
		// Defined in derived classes to handle the nuts
		// and bolts of grabbing a frame from a given
		// source.  preLock happens before the mutex
		// while postLock happens inside it
		bool postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud);
		bool postLockFrameNumber(int framenumber);

	private:
		void deleteInputPointers(void);
		bool openSerializeInput(const char *filename, bool portable, bool conpressed);
		void update(void);

		// frame_ is the most recent frame grabbed from 
		// the camera
		// prevGetFrame_ is the last frame returned from
		// getFrame().  If paused, code needs to keep returning
		// this frame rather than getting a new one from frame_
		cv::Mat      frame_;
		cv::Mat      depth_;
		cv::Mat      prevGetFrame_;
		cv::Mat      prevGetDepth_;

		// Hack up a way to save zed data - serialize both
		// BGR frame and depth frame
		std::ifstream *serializeIn_;
		boost::iostreams::filtering_streambuf<boost::iostreams::input> *filtSBIn_;
		boost::archive::binary_iarchive *archiveIn_;
		portable_binary_iarchive *portableArchiveIn_;
};
