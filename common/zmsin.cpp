// ZMS file is home-brewed serialization format
// which just dumps raw a image and depth Mat data to a file.
// Apply a light bit of compression because
// the files will get out of hand quickly otherwise
// Initial versions of these files were non-portable
// but later versions were changed to be useable
// on both ARM and x86.  Handle loading both types,
// at least for the time being
#include <iostream>
#include <fstream>
#include <limits>
#include "zmsin.hpp"

#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cvMatSerialize.hpp"
#include "ZvSettings.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

ZMSIn::ZMSIn(const char *inFileName, ZvSettings *settings) :
	SyncIn(settings),
	serializeIn_(NULL),
	filtSBIn_(NULL),
	archiveIn_(NULL),
	portableArchiveIn_(NULL)
{
	width_ = 0;
	height_ = 0;
	// Grab the first frame to figure out image size
	cerr << "Loading " << inFileName << " for reading" << endl;
	bool loaded = false;
	if (openSerializeInput(inFileName, true, true)  ||
		openSerializeInput(inFileName, false, true) || 
	    openSerializeInput(inFileName, true, false) ||
		openSerializeInput(inFileName, false, false))
	{
		loaded = true;
		try
		{
			if (archiveIn_)
				*archiveIn_ >> frame_ >> depth_;
			else
				*portableArchiveIn_ >> frame_ >> depth_;
		}
		catch (const std::exception &e)
		{
			loaded = false;
		}
	}

	if (!loaded)
	{
		cerr << "ZMSIn(): Could not open " << inFileName << " for reading" << endl;
		deleteInputPointers();
		return;
	}

	width_  = frame_.cols;
	height_ = frame_.rows;

	// Reopen the file so callers can get the first frame
	if (!openSerializeInput(inFileName, archiveIn_ == NULL, true) &&
	    !openSerializeInput(inFileName, archiveIn_ == NULL, false))
	{
		cerr << "ZMSIn() : Could not reopen " << inFileName << " for reading" << endl;
		return;
	}

	while (height_ > 700)
	{
		width_  /= 2;
		height_ /= 2;
	}
	startThread();
}

// Input needs 3 things. First is a standard ifstream to read from
// Next is an (optional) filtered stream buffer. This is used to
// uncompress on the fly - uncompressed files take up way too
// much space. Last item is the actual boost binary archive template
// If all three are opened, return true. If not, delete and set to
// NULL all pointers related to serialized Input
bool ZMSIn::openSerializeInput(const char *inFileName, bool portable, bool compressed)
{
	deleteInputPointers();
	serializeIn_ = new ifstream(inFileName, ios::in | ios::binary);
	if (!serializeIn_ || !serializeIn_->is_open())
	{
		cerr << "Could not open ifstream(" << inFileName << ")" << endl;
		deleteInputPointers();
		return false;
	}

	filtSBIn_= new boost::iostreams::filtering_streambuf<boost::iostreams::input>;
	if (!filtSBIn_)
	{
		cerr << "Could not create filtering_streambuf<input>" << endl;
		deleteInputPointers();
		return false;
	}
	if (compressed)
		filtSBIn_->push(boost::iostreams::zlib_decompressor());
	filtSBIn_->push(*serializeIn_);
	if (portable)
	{
		try
		{
			portableArchiveIn_ = new portable_binary_iarchive(*filtSBIn_);
		}
		catch (std::exception &e)
		{
			portableArchiveIn_ = NULL;
		}
		if (!portableArchiveIn_)
		{
			cerr << "Could not create new portable_binary_iarchive" << endl;
			deleteInputPointers();
			return false;
		}
	}
	else
	{
		try
		{
			archiveIn_ = new boost::archive::binary_iarchive(*filtSBIn_);
		}
		catch (std::exception &e)
		{
			portableArchiveIn_ = NULL;
		}
		if (!archiveIn_)
		{
			cerr << "Could not create new binary_iarchive" << endl;
			deleteInputPointers();
			return false;
		}
	}
	return true;
}

// Helper to easily delete and NULL out input file pointers
void ZMSIn::deleteInputPointers(void)
{
	if (archiveIn_)
	{
		delete archiveIn_;
		archiveIn_ = NULL;
	}
	if (portableArchiveIn_)
	{
		delete portableArchiveIn_;
		portableArchiveIn_ = NULL;
	}
	if (filtSBIn_)
	{
		delete filtSBIn_;
		filtSBIn_ = NULL;
	}
	if (serializeIn_)
	{
		delete serializeIn_;
		serializeIn_ = NULL;
	}
}


ZMSIn::~ZMSIn()
{
	stopThread();
	deleteInputPointers();
}


bool ZMSIn::isOpened(void) const
{
	return archiveIn_ || portableArchiveIn_;
}


bool ZMSIn::postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	cloud.clear();
	// Ugly try-catch to detect EOF
	try
	{
		if (archiveIn_)
			*archiveIn_ >> frame >> depth;
		else
			*portableArchiveIn_ >> frame >> depth;
	}
	catch (const std::exception &e)
	{
		// EOF reached.  Signal this by sending
		// an empty frame to getFrame.
		return false;
	}
	return true;
}


// Can't randomly seek in ZMS yet
// TOOD : if moving forward, read frames sequentially?
//        if moving backwards, reopen then read forward sequentially?
bool ZMSIn::postLockFrameNumber(int framenumber)
{
	(void) framenumber;
	return false;

}


// Use hard-coded values takes from SN*.conf files
CameraParams ZMSIn::getCameraParams(void) const
{
	float hFovDegrees;
	if (height_ == 480) // TODO : redo this now that VGA is widescreen VGA instead
		hFovDegrees = 51.3;
	else
		hFovDegrees = 105.; // hope all the HD & 2k res are the same
	float hFovRadians = hFovDegrees * M_PI / 180.0;

	CameraParams params;
	params.fov = Point2f(hFovRadians, hFovRadians * (float)height_ / (float)width_);
	// Take a guess based on acutal values from one of our cameras
	if (height_ == 480)
	{
		params.fx = 350.075;
		params.fy = 350.075;
		params.cx = 332.289;
		params.cy = 186.061;
	}
	else if ((width_ == 1280) || (width_ == 640)) // 720P normal or pyrDown 1x
	{
		params.fx = 700.548;
		params.fy = 700.548;
		params.cx = 647.35  / (1280 / width_);
		params.cy = 369.472 / (1280 / width_);
	}
	else if ((width_ == 1920) || (width_ == 960)) // 1920 downscaled
	{
		params.fx = 1401.1;
		params.fy = 1401.1;
		params.cx = 977.701 / (1920 / width_); // Is this correct - downsized
		params.cy = 561.944 / (1920 / width_); // image needs downsized cx?
	}
	else if ((width_ == 2208) || (width_ == 1104)) // 2208 downscaled
	{
		params.fx = 1401.1;
		params.fy = 1401.14;
		params.cx = 1121.74 / (2208 / width_);
		params.cy = 642.944 / (2208 / width_);
	}
	else
	{
		// This should never happen
		params.fx = 0;
		params.fy = 0;
		params.cx = 0;
		params.cy = 0;
	}
	return params;
}

int ZMSIn::frameCount(void) const
{
	return 0;
}
