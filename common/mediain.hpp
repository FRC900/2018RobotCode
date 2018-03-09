// Base class for all input - cameras, videos, stills
// Has the basics all inputs use - current frame number,
// width& height, and a timestamp the current frame was aquired
// Other methods are declared as pure virtual which forces
// derived classes to implement them with the particulars
// of that input type.
#pragma once

#include <opencv2/core/core.hpp>
#include <tinyxml2.h>
#include <pcl/common/common_headers.h>

#include "cameraparams.hpp"
#include "frameticker.hpp"
#include "ZvSettings.hpp"
//#include "navXTimeSync/AHRS.h"

// Base class for input.  Derived classes are cameras, videos, etc
class MediaIn
{
	public:
		MediaIn(ZvSettings *settings);
		virtual ~MediaIn() {}

		// Make class non-copyable
		MediaIn(const MediaIn &mediain) = delete;
		MediaIn &operator=(const MediaIn &mediain) = delete;

		// These should be implemented by each derived class
		// Include defaults here which return false just in case
		virtual bool isOpened(void) const;

		// Get data from current frame. Note the pause parameter. If it is
		// false it returns the newest data from the input. If pause
		// is true is returns data corresponding to the data returned last
		// time these functions were called with pause == false. That way
		// if, say, video is paused multiple calls to getFrame will all return
		// the same data. Likewise, a call to getDepth() will return depth data
		// which lines up with the last call to getFrame() rather than newer
		// data from a random frame.
		virtual bool getFrame(cv::Mat &frame, bool pause = false);
		virtual bool getFrame(cv::Mat &frame, cv::Mat &depth, bool pause = false);
		virtual bool getFrame(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause = false);
		virtual bool getDepth(cv::Mat &depth, bool pause = true);
		virtual bool getPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause = true);

		// Image size
		unsigned int width() const;
		unsigned int height() const;

		// How many frames?
		virtual int frameCount(void) const;

		// Set current frame number
		virtual void frameNumber(int frameNumber);

		// Get frame number and the time that frame was
		// captured. Former only makes sense for video
		// input and the latter for live camera feeds
		int frameNumber(void) const;
		long long timeStamp(void) const;

		// Input FPS for live camera input
		virtual float FPS(void) const;

		// Camera parameters - fov, focal length, etc.
		virtual CameraParams getCameraParams(void) const;

	protected:
		// Width and height of input frame
		unsigned int width_;
		unsigned int height_;

		// Saved settings for this input type
		ZvSettings *settings_;

		// Disable point cloud processing until 
		// a getFrame() or getPointCloud() call
		// asks for it - speeds up processing for
		// cases which don't use it
		bool usePointCloud_;

		void setTimeStamp(long long timeStamp = -1);
		void lockTimeStamp(void);
		void setFrameNumber(int frameNumber);
		void incFrameNumber(void);
		void lockFrameNumber(void);
		void FPSmark(void);

		virtual bool loadSettings(void);
		virtual bool saveSettings(void) const;
		virtual std::string getClassName() const { return "MediaIn"; }

	private:
		// Maintain two sets of frame numbers and time stamps.
		// The locked version corresponds to the frame that was
		// current the last time getFrame was called.  The other
		// one is updated for each input frame. Since the update()
		// threads can run at a different speed than getFrame is called,
		// this lets the code maintain the correct values associated 
		// with each - i.e. multiple calls to a paused getFrame() will
		// return the same locked* value even as update() changes the
		// non-locked versions in a separate thread
		int       frameNumber_;
		int       lockedFrameNumber_;
		long long timeStamp_;
		long long lockedTimeStamp_;
		FrameTicker frameTicker;
		//AHRS NavXHandle;
};
