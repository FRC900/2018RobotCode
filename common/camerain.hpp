// Class to handle input from generic camera sources
// Uses OpenCV's VideoCapture interface to control some
// basic settings but there's not too much interesting
// to do with contrast/brightness/etc.
// Code runs an update thread in the background which 
// constantly polls the camera.  Calls to getFrame()
// return the most recent data (unless pause is set - then
// the last frame called with pause==false is returned)
#pragma once

#include "asyncin.hpp"

class ZvSettings;

class CameraIn : public AsyncIn
{
	public:
		CameraIn(int stream = -1, ZvSettings *settings = NULL);
		~CameraIn();

		bool isOpened(void) const;

	protected:
		// Defined in derived classes to handle the nuts
		// and bolts of grabbing a frame from a given
		// source.  preLock happens before the mutex
		// while postLock happens inside it
		bool preLockUpdate(void);
		bool postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud);

	private:
		double           fps_;  // FPS requested/set for camera
		cv::VideoCapture cap_;  // the input camera object
		cv::Mat          localFrame_; // local frame buffer

		std::string getClassName() const { return "CameraIn"; } 
		bool loadSettings(void);
		bool saveSettings(void) const;

		// Input size before scaling down
		int realWidth_;
		int realHeight_;
};
