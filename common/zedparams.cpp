#include <iostream>
#include "zedparams.hpp"

using namespace std;

ZedParams::ZedParams(void)
{
}

#ifdef ZED_SUPPORT

using namespace sl;

CameraParams ZedParams::get(void) const
{
	return params_;
}

// For whatever reason, getParameters calls grab()
// This causes problems with the update() thread
// Could mutex protect this, but the easier way
// is to just set it once in the constructor once
// a Zed object is opened and then from there on return
// the results 
void ZedParams::init(sl::Camera &zed, bool left)
{
	CameraParameters zedp = left ?
		zed.getCameraInformation().calibration_parameters.left_cam:
		zed.getCameraInformation().calibration_parameters.right_cam;

	// TODO : ZED SDK added hFOV and vFOV fields to 
	// CamParameters - use them
    Resolution image_size = zed.getResolution();
	unsigned height = image_size.height;
	unsigned width  = image_size.width;
	// Pick FOV based on 4:3 or 16:9 aspect ratio
	float hFovDegrees;
	if (height == 480) // 640x480. TODO :: widescreen VGA?
		hFovDegrees = 51.3;
	else
		hFovDegrees = 105.; // hope all the HD & 2k res are the same
	float hFovRadians = hFovDegrees * M_PI / 180.0;

	// Convert from ZED-specific to generic
	// params data type
	params_.fov = cv::Point2f(hFovRadians, hFovRadians * (float)height / (float)width);
	params_.fx = zedp.fx;
	params_.fy = zedp.fy;
	params_.cx = zedp.cx;
	params_.cy = zedp.cy;
	for (size_t i = 0; i < 5; i++)
		params_.disto[i] = zedp.disto[i];
}

#endif
