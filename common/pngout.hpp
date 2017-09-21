#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "mediaout.hpp"

// Class to write MJPEG PNG files using
// OpenCV's VideoWriter class
class PNGOut : public MediaOut
{
	public:
		PNGOut(const char *outFile);
		~PNGOut();

	private :
		bool openNext(int fileCounter);
		bool write(const cv::Mat &frame, const cv::Mat &depth);

		cv::Size         size_;
		std::string      fileName_;
};
