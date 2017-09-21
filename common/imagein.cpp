// Class to handle still images (JPG, PNG, etc) as input.
// This is pretty simple. Since there's just a single image,
// read it in and return it every time getFrame is called

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "imagein.hpp"
#include "ZvSettings.hpp"

using namespace cv;

ImageIn::ImageIn(const char *inpath, ZvSettings *settings) :
	MediaIn(settings)
{
	imread(inpath).copyTo(image_);
	if (image_.empty())
	{
		std::cerr << "Could not open image file " << inpath << std::endl;
		return;
	}
	while (image_.rows > 700)
		pyrDown(image_, image_);

	width_ = image_.cols;
	height_ = image_.rows;
}

bool ImageIn::isOpened(void) const
{
	return image_.empty();
}

bool ImageIn::getFrame(Mat &frame, bool pause)
{
	(void)pause;
	if (image_.empty())
		return false;
	frame = image_.clone();
	return true;
}

// Images have only 1 "frame"
int ImageIn::frameCount(void) const
{
	return 1;
}

