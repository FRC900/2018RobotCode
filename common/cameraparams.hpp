#pragma once
// Mimic ZED camera parameters instead of using them - allows
// for targets without ZED support to still get this info
// for other cameras
#include <opencv2/core/core.hpp>
class CameraParams
{
	public:
		CameraParams(void);

		cv::Point2f fov;
		float       fx;
		float       fy;
		float       cx;
		float       cy;
		double      disto[5];
};


