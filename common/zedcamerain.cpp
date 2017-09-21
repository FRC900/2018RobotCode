#include <iostream>
#include "zedcamerain.hpp"
using namespace std;

#ifdef ZED_SUPPORT
#include <opencv2/imgproc/imgproc.hpp>

#include "cvMatSerialize.hpp"
#include "ZvSettings.hpp"

using namespace cv;
using namespace sl;

void zedBrightnessCallback(int value, void *data);
void zedContrastCallback(int value, void *data);
void zedHueCallback(int value, void *data);
void zedSaturationCallback(int value, void *data);
void zedGainCallback(int value, void *data);
void zedExposureCallback(int value, void *data);
void zedWhiteBalanceCallback(int value, void *data);

ZedCameraIn::ZedCameraIn(bool gui, ZvSettings *settings) :
	AsyncIn(settings),
	brightness_(2),
	contrast_(6),
	hue_(7),
	saturation_(4),
	gain_(1),
	exposure_(1), // Should set exposure = -1 => auto exposure/auto-gain
	whiteBalance_(0), // Auto white-balance?
	opened_(false)
{
	if (!Camera::isZEDconnected()) // Open an actual camera for input
	{
		cerr << "ZED camera not found" << endl;
		return;
	}

	InitParameters parameters;
	parameters.camera_resolution = RESOLUTION_HD720;

	// Runs cleanly at 30 FPS on TX1, doesn't init
	// at 60FPS?
	parameters.camera_fps = 60;

	parameters.depth_mode = DEPTH_MODE_PERFORMANCE;
	parameters.coordinate_units = UNIT_METER;
	parameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP ; // Y pointing out and Z pointing up. Only used for point cloud?
	parameters.sdk_verbose = 1;
	parameters.camera_buffer_count_linux = 4; // default : experiment with lower

	// The computation time is affected by the value,
	// exponentially. The closer it gets the longer the
	// disparity search will take. In case of limited
	// computation power, consider increasing the value.
	//parameters.depth_minimum_distance = 1; // in coordinate_units, so ~1m

	// init computation mode of the zed
	ERROR_CODE err = zed_.open(parameters);
	// Quit if an error occurred
	if (err != SUCCESS)
	{
		cout << errorCode2str(err) << endl;
		return;
	}
	opened_ = true;
	zed_.setCameraFPS(parameters.camera_fps);
    // Create sl and cv Mat to get ZED left image and depth image
    // Best way of sharing sl::Mat and cv::Mat :
    // Create a sl::Mat and then construct a cv::Mat using the ptr to sl::Mat data.
    Resolution image_size = zed_.getResolution();
	width_  = image_size.width;
	height_ = image_size.height;
    localFrameZed_.alloc(image_size, MAT_TYPE_8U_C4); // Create a sl::Mat to handle Left image
    localFrameRGBA_ = cv::Mat(height_, width_, CV_8UC4, localFrameZed_.getPtr<sl::uchar1>(sl::MEM_CPU)); // Create an OpenCV Mat that shares sl::Mat buffer

    localDepthZed_.alloc(image_size, MAT_TYPE_32F_C1);
    localDepth_ = cv::Mat(height_, width_, CV_32FC1, localDepthZed_.getPtr<sl::uchar1>(sl::MEM_CPU));

	if (!loadSettings())
		cerr << "Failed to load ZedCameraIn settings from XML" << endl;

	zedBrightnessCallback(brightness_, this);
	zedContrastCallback(contrast_, this);
	zedHueCallback(hue_, this);
	zedSaturationCallback(saturation_, this);
	zedGainCallback(gain_, this);
	zedExposureCallback(exposure_, this);
	zedWhiteBalanceCallback(whiteBalance_, this);

	cout << "brightness_ = " << zed_.getCameraSettings(CAMERA_SETTINGS_BRIGHTNESS) << endl;
	cout << "contrast_ = " << zed_.getCameraSettings(CAMERA_SETTINGS_CONTRAST) << endl;
	cout << "hue_ = " << zed_.getCameraSettings(CAMERA_SETTINGS_HUE) << endl;
	cout << "saturation_ = " << zed_.getCameraSettings(CAMERA_SETTINGS_SATURATION) << endl;
	cout << "gain_ = " << zed_.getCameraSettings(CAMERA_SETTINGS_GAIN) << endl;
	cout << "exposure_ = " << zed_.getCameraSettings(CAMERA_SETTINGS_EXPOSURE) << endl;
	cout << "whiteBalance_ = " << zed_.getCameraSettings(CAMERA_SETTINGS_WHITEBALANCE) << endl;
	if (gui)
	{
		cv::namedWindow("Adjustments", CV_WINDOW_NORMAL);
		cv::createTrackbar("Brightness", "Adjustments", &brightness_, 9, zedBrightnessCallback, this);
		cv::createTrackbar("Contrast", "Adjustments", &contrast_, 9, zedContrastCallback, this);
		cv::createTrackbar("Hue", "Adjustments", &hue_, 12, zedHueCallback, this);
		cv::createTrackbar("Saturation", "Adjustments", &saturation_, 9, zedSaturationCallback, this);
		cv::createTrackbar("Gain", "Adjustments", &gain_, 101, zedGainCallback, this);
		cv::createTrackbar("Exposure", "Adjustments", &exposure_, 102, zedExposureCallback, this);
		cv::createTrackbar("WhiteBalance", "Adjustments", &whiteBalance_, 39, zedWhiteBalanceCallback, this);
	}

	while (height_ > 700)
	{
		width_  /= 2;
		height_ /= 2;
	}

	params_.init(zed_, true);
	startThread();
}


ZedCameraIn::~ZedCameraIn()
{
	if (!saveSettings())
		cerr << "Failed to save ZedCameraIn settings to XML" << endl;
	stopThread();
	zed_.close();
	opened_ = false;
}


bool ZedCameraIn::loadSettings(void)
{
	if (settings_) {
		settings_->getInt(getClassName(), "brightness",   brightness_);
		settings_->getInt(getClassName(), "contrast",     contrast_);
		settings_->getInt(getClassName(), "hue",          hue_);
		settings_->getInt(getClassName(), "saturation",   saturation_);
		settings_->getInt(getClassName(), "gain",         gain_);
		settings_->getInt(getClassName(), "exposure",     exposure_);
		settings_->getInt(getClassName(), "whiteBalance", whiteBalance_);
		return true;
	}
	return false;
}


bool ZedCameraIn::saveSettings(void) const
{
	if (settings_) {
		settings_->setInt(getClassName(), "brightness",   brightness_);
		settings_->setInt(getClassName(), "contrast",     contrast_);
		settings_->setInt(getClassName(), "hue",          hue_);
		settings_->setInt(getClassName(), "saturation",   saturation_);
		settings_->setInt(getClassName(), "gain",         gain_);
		settings_->setInt(getClassName(), "exposure",     exposure_);
		settings_->setInt(getClassName(), "whiteBalance", whiteBalance_);
		settings_->save();
		return true;
	}
	return false;
}


bool ZedCameraIn::isOpened(void) const
{
	return opened_;
}


CameraParams ZedCameraIn::getCameraParams(void) const
{
	return params_.get();
}


// Code for grabbing data from camera into local buffers
// Doesn't need a lock since these aren't shared buffers
// with the main thread
bool ZedCameraIn::preLockUpdate(void)
{
	const bool left = true;
	int badReadCounter = 0;

	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_STANDARD; // Use STANDARD sensing mode
	runtime_parameters.enable_depth = true;
	runtime_parameters.enable_point_cloud = usePointCloud_;

	while (zed_.grab(runtime_parameters) != SUCCESS)
	{
		// grab() will return true if the next
		// frame isn't ready yet
		boost::this_thread::interruption_point();
		// Wait a bit to see if the next
		// frame shows up
		usleep(5000);
		// Try to grab a bunch of times before
		// bailing out and failing
		if (++badReadCounter == 100)
			return false;
	}

	zed_.retrieveImage(localFrameZed_, left ? VIEW_LEFT : VIEW_RIGHT);
#if 0
	if (localFrameZed_.getMemoryType() == MEM_GPU)
		localFrameZed_.updateCPUfromGPU();
#endif
	// TODO :: See about running the cvtColor in GPU space?
	cvtColor(localFrameRGBA_, localFrameRGB_, CV_RGBA2RGB);

	zed_.retrieveMeasure(localDepthZed_, MEASURE_DEPTH);
#if 0
	if (localDepthZed_.getMemoryType() == MEM_GPU)
		localDepthZed_.updateCPUfromGPU();
#endif

	localCloud_.clear();
	if (usePointCloud_)
	{
		zed_.retrieveMeasure(localCloudZed_, MEASURE_XYZRGBA);
		float *pCloud = localCloudZed_.getPtr<float>();
		for (int i = 0; i < (localDepth_.rows * localDepth_.cols); i++)
		{
			if (isValidMeasure(pCloud[i * 4]))
			{
				pcl::PointXYZRGB pt;
				pt.x = pCloud[i * 4 + 0];
				pt.y = pCloud[i * 4 + 1];
				pt.z = pCloud[i * 4 + 2];
				float color = pCloud[i * 4 + 3];
				// Color conversion (RGBA as float32 -> RGB as uint32)
				uint32_t color_uint = *(uint32_t*) &color;
				unsigned char* color_uchar = (unsigned char*) &color_uint;
				color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
				pt.rgb = *reinterpret_cast<float*> (&color_uint);
				localCloud_.push_back(pt);
			}
		}
	}

	return true;
}


bool ZedCameraIn::postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	localFrameRGB_.copyTo(frame);
	localDepth_.copyTo(depth);
	cloud = localCloud_;

	return true;
}


void zedBrightnessCallback(int value, void *data)
{
    ZedCameraIn *zedPtr = static_cast<ZedCameraIn *>(data);
	zedPtr->brightness_ = value;
	if (zedPtr->opened_)
		zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_BRIGHTNESS, value - 1, value == 0);
}


void zedContrastCallback(int value, void *data)
{
    ZedCameraIn *zedPtr = static_cast<ZedCameraIn *>(data);
	zedPtr->contrast_ = value;
	if (zedPtr->opened_)
		zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_CONTRAST, value - 1, value == 0);
}


void zedHueCallback(int value, void *data)
{
    ZedCameraIn *zedPtr = static_cast<ZedCameraIn *>(data);
	zedPtr->hue_ = value;
	if (zedPtr->opened_)
		zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_HUE, value - 1, value == 0);
}


void zedSaturationCallback(int value, void *data)
{
    ZedCameraIn *zedPtr = static_cast<ZedCameraIn *>(data);
	zedPtr->saturation_ = value;
	if (zedPtr->opened_)
		zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_SATURATION, value - 1, value == 0);
}


void zedGainCallback(int value, void *data)
{
    ZedCameraIn *zedPtr = static_cast<ZedCameraIn *>(data);
	zedPtr->gain_ = value;
	if (zedPtr->opened_)
		zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_GAIN, value - 1, value == 0);
}


void zedExposureCallback(int value, void *data)
{
    ZedCameraIn *zedPtr = static_cast<ZedCameraIn *>(data);
	zedPtr->exposure_ = value;
	if (zedPtr->opened_)
		zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, value - 1, value == 0);
}


void zedWhiteBalanceCallback(int value, void *data)
{
    ZedCameraIn *zedPtr = static_cast<ZedCameraIn *>(data);
	zedPtr->whiteBalance_= value;
	if (zedPtr->opened_)
	{
		// Defines the color temperature control. Affected
		// value should be between 2800 and 6500 with a step
		// of 100. A value of -1 set the AWB ( auto white balance),
		// as the boolean parameter (default) does. 
		if (value == 0)
		{
			zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_AUTO_WHITEBALANCE, true);
			zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_WHITEBALANCE, -1);
		}
		else
		{
			zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_AUTO_WHITEBALANCE, false);
			zedPtr->zed_.setCameraSettings(CAMERA_SETTINGS_WHITEBALANCE, value * 100 + 2700);
		}
	}
}

#else

ZedCameraIn::ZedCameraIn(bool gui, ZvSettings *settings) :
	AsyncIn(settings)
{
	(void)gui;
}

ZedCameraIn::~ZedCameraIn()
{
}


bool ZedCameraIn::preLockUpdate(void)
{
	return true;
}


bool ZedCameraIn::postLockUpdate(cv::Mat &frame, cv::Mat &depth,pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	(void)frame;
	(void)depth;
	(void)cloud;
	return false;
}
#endif

