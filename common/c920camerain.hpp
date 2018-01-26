// Similar to camerain, but exposes all of the camera controls
// specific to the C920 camera
#pragma once
#include <opencv2/core/core.hpp>

#include "asyncin.hpp"

#ifdef __linux__
#include "C920Camera.h"
#endif

class ZvSettings;

// Code specific for C920 camera. We have lots of
// extra controls avaiable for this, so use it if
// possible
class C920CameraIn : public AsyncIn
{
	public:
		C920CameraIn(int stream = -1, bool gui = false, ZvSettings *settings = NULL);
		~C920CameraIn();

#ifdef __linux__  // Special C920 support only works under linux
		bool isOpened(void) const;

		CameraParams getCameraParams(void) const;

	protected:
		// Defined in derived classes to handle the nuts
		// and bolts of grabbing a frame from a given
		// source.  preLock happens before the mutex
		// while postLock happens inside it
		bool preLockUpdate(void);
		bool postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud);

	private:
		// The camera object itself
		v4l2::C920Camera  camera_;

		// Frame buffer
		cv::Mat           localFrame_;

		// Various camera settings
		int               brightness_;
		int               contrast_;
		int               saturation_;
		int               sharpness_;
		int               gain_;
		int               focus_;
		int               autoExposure_;
		int               backlightCompensation_;
		int               whiteBalanceTemperature_;
		v4l2::CaptureSize captureSize_;
		v4l2::CaptureFPS  captureFPS_;

		bool initCamera(bool gui);
		bool loadSettings(void);
		bool saveSettings(void) const;
		std::string getClassName() const { return "C920CameraIn"; }

		// Mark these as friends so they can access private class data
		friend void brightnessCallback(int value, void *data);
		friend void contrastCallback(int value, void *data);
		friend void saturationCallback(int value, void *data);
		friend void sharpnessCallback(int value, void *data);
		friend void gainCallback(int value, void *data);
		friend void autoExposureCallback(int value, void *data);
		friend void backlightCompensationCallback(int value, void *data);
		friend void whiteBalanceTemperatureCallback(int value, void *data);
		friend void focusCallback(int value, void *data);
#endif
};
