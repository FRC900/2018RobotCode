#include <iostream>
#include "zedsvoin.hpp"
using namespace std;

#ifdef ZED_SUPPORT
using namespace cv;
using namespace sl;

ZedSVOIn::ZedSVOIn(const char *inFileName, ZvSettings *settings) :
	SyncIn(settings),
	opened_(false),
	frames_(-1)
{
	InitParameters parameters;
	parameters.camera_resolution = RESOLUTION_HD720;

	// Ball detection runs at ~10 FPS on Jetson
	// so run camera capture more slowly
	parameters.camera_fps =
#ifdef IS_JETSON
		15;
#else
		30;
#endif

	parameters.depth_mode = DEPTH_MODE_PERFORMANCE;
	parameters.coordinate_units = UNIT_MILLIMETER;
	parameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP ; // Y pointing out and Z pointing up. Only used for point cloud?
	parameters.sdk_verbose = 1;
	parameters.camera_buffer_count_linux = 4; // default : experiment with lower
	parameters.svo_input_filename = inFileName;

	// init computation mode of the zed
	ERROR_CODE err = zed_.open(parameters);

	// Quit if an error occurred
	if (err != SUCCESS)
	{
		cout << errorCode2str(err) << endl;
		return;
	}
	opened_ = true;
	frames_ = zed_.getSVONumberOfFrames();
    // Create sl and cv Mat to get ZED left image and depth image
    // Best way of sharing sl::Mat and cv::Mat :
    // Create a sl::Mat and then construct a cv::Mat using the ptr to sl::Mat data.
    Resolution image_size = zed_.getResolution();
	width_  = image_size.width;
	height_ = image_size.height;
    localFrameZed_.alloc(image_size, MAT_TYPE_8U_C4); // Create a sl::Mat to handle Left image
    localFrame_   = cv::Mat(height_, width_, CV_8UC4, localFrameZed_.getPtr<sl::uchar1>(sl::MEM_CPU)); // Create an OpenCV Mat that shares sl::Mat buffer

    localDepthZed_.alloc(image_size, MAT_TYPE_32F_C1);
    localDepth_    = cv::Mat(height_, width_, CV_32FC1, localDepthZed_.getPtr<sl::uchar1>(sl::MEM_CPU));

	params_.init(zed_, true);
	startThread();

	while (height_ > 700)
	{
		width_  /= 2;
		height_ /= 2;
	}
}

ZedSVOIn::~ZedSVOIn()
{
	stopThread();
	zed_.close();
	opened_ = false;
}

bool ZedSVOIn::isOpened(void) const
{
	return opened_;
}


CameraParams ZedSVOIn::getCameraParams(void) const
{
	return params_.get();
}


bool ZedSVOIn::postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	if (!zed_.isOpened())
		return false;

	if (zed_.grab() != SUCCESS)
		return false;
	const bool left = true;

	zed_.retrieveImage(localFrameZed_, left ? VIEW_LEFT : VIEW_RIGHT);
	if (localFrameZed_.getMemoryType() == MEM_GPU)
		localFrameZed_.updateCPUfromGPU();
	cvtColor(localFrame_, frame, CV_RGBA2RGB);

	zed_.retrieveMeasure(localDepthZed_, MEASURE_DEPTH);
	if (localDepthZed_.getMemoryType() == MEM_GPU)
		localDepthZed_.updateCPUfromGPU();
	localDepth_.copyTo(depth);

	zed_.retrieveMeasure(localCloudZed_, MEASURE_XYZRGBA);
	float *pCloud = localCloudZed_.getPtr<float>();
	cloud.clear();
	for (int i = 0; i < (depth.rows * depth.cols); i++)
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
			cloud.push_back(pt);
		}
	}

	return true;
}


bool ZedSVOIn::postLockFrameNumber(int framenumber)
{
	if (!zed_.isOpened())
	   return false;
	zed_.setSVOPosition(framenumber);
	return true;
}


int ZedSVOIn::frameCount(void) const
{
	return frames_;
}


#else

ZedSVOIn::ZedSVOIn(const char *inFileName, ZvSettings *settings) :
	SyncIn(settings)
{
	(void)inFileName;
}


ZedSVOIn::~ZedSVOIn()
{
}


bool ZedSVOIn::postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	(void)frame;
	(void)depth;
	(void)cloud;
	return true;
}


bool ZedSVOIn::postLockFrameNumber(int framenumber) 
{
	(void)framenumber;
	return 0;
}
#endif
