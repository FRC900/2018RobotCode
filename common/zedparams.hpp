#pragma once

#ifdef ZED_SUPPORT
#include <sl/Camera.hpp>
#include "cameraparams.hpp"
#endif

class ZvSettings;

class ZedParams
{
	public:
		ZedParams(void);

#ifdef ZED_SUPPORT
		void init(sl::Camera &zed, bool left);
		CameraParams get(void) const;

	private:
		CameraParams params_;

#endif
};
