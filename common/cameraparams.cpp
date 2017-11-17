#include "cameraparams.hpp"

CameraParams::CameraParams(void) :
	fov(51.3 * M_PI / 180., 51.3 * 480. / 640. * M_PI / 180.), // Default to zed params?
	fx(0),
	fy(0),
	cx(0),
	cy(0)
{
	for (size_t i = 0; i < sizeof(disto)/sizeof(disto[0]); i++)
		disto[i] = 0.0;
}

