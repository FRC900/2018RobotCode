#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "syncin.hpp"
#include "ZvSettings.hpp"

using namespace cv;

SyncIn::SyncIn(ZvSettings *settings) :
	MediaIn(settings),
	frameReady_(false)  // signal update() to load a new frame immediately
{
}

void SyncIn::startThread(void)
{
	thread_ = boost::thread(&SyncIn::update, this);
}

void SyncIn::stopThread(void)
{
	thread_.interrupt();
	thread_.join();
}

// Read the next frame from the input file.  Store the
// read frame in frame_.
// The code is designed not to skip any input frames,
// so if the data stored in frame_ hasn't been read
// in getFrame yet, update() will loop until it has
// before overwriting it.
void SyncIn::update(void)
{
	// Loop until an empty frame is read - 
	// this should identify EOF
	do
	{
		// If the frame read from the last update()
		// call hasn't been used yet, loop here
		// until it has been. This will prevent
		// the code from reading multiple frames
		// in the time it takes to process one and
		// skipping some video in the process
		boost::mutex::scoped_lock guard(mtx_);
		while (frameReady_)
			condVar_.wait(guard);

		if (postLockUpdate(frame_, depth_, cloud_))
		{
			setTimeStamp();
			incFrameNumber();
			while (frame_.rows > 700)
			{
				pyrDown(frame_, frame_);
				if (!depth_.empty())
					pyrDown(depth_, depth_);
			}
		}
		else
		{
			frame_ = Mat();
			depth_ = Mat();
		}

		// Let getFrame know that a frame is ready
		// to be read / processed
		frameReady_ = true;
		condVar_.notify_all();
	}
	while (!frame_.empty());
}

// Used to copy data read from input
// into buffers used to return them to callers
// of getFrame() and the like
bool SyncIn::copyBuffers(void)
{
	// Wait until a valid frame is in frame_
	boost::mutex::scoped_lock guard(mtx_);
	while (!frameReady_)
		condVar_.wait(guard);

	if (frame_.empty())
	{
		pausedFrame_ = Mat();
		pausedDepth_= Mat();
		return false;
	}

	// Save copies of the previously
	// returned frame's data. This lets
	// the code return it again if paused
	frame_.copyTo(pausedFrame_);
	depth_.copyTo(pausedDepth_);
	pausedCloud_ = cloud_;
	lockTimeStamp();
	lockFrameNumber();

	// Let update() know that getFrame has copied
	// the current frame out of frame_
	frameReady_ = false;
	condVar_.notify_all();

	// Release the mutex so that update() can
	// start getting the next frame while the
	// current one is returned and processed
	// in the main thread.
	return true;
}


bool SyncIn::getFrame(Mat &frame, bool pause)
{
	if (!isOpened())
		return false;

	// If not paused, copy the next frame data from
	// frame_. This is the Mat holding the next
	// frame read from the video that update()
	// fills in a separate thread
	// Read if pausedFrame_.empty() to handle
	// case where pause == true on first call
	// to getFrame - without it nothing will ever
	// be read
	if (!pause || pausedFrame_.empty())
		if (!copyBuffers())
			return false;

	if (pausedFrame_.empty())
		return false;
	pausedFrame_.copyTo(frame);
	return true;
}


bool SyncIn::getFrame(Mat &frame, Mat &depth, bool pause)
{
	if (!isOpened())
		return false;

	// If not paused, copy the next frame data from
	// frame_. This is the Mat holding the next
	// frame read from the video that update()
	// fills in a separate thread
	// Read if pausedFrame_.empty() to handle
	// case where pause == true on first call
	// to getFrame - without it nothing will ever
	// be read
	if (!pause || pausedFrame_.empty())
		if (!copyBuffers())
			return false;

	if (pausedFrame_.empty())
		return false;
	pausedFrame_.copyTo(frame);
	pausedDepth_.copyTo(depth);
	return true;
}

bool SyncIn::getFrame(Mat &frame, Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause)
{
	if (!isOpened())
		return false;

	usePointCloud_ = true;

	// If not paused, copy the next frame data from
	// frame_. This is the Mat holding the next
	// frame read from the video that update()
	// fills in a separate thread
	// Read if pausedFrame_.empty() to handle
	// case where pause == true on first call
	// to getFrame - without it nothing will ever
	// be read
	if (!pause || pausedFrame_.empty())
		if (!copyBuffers())
			return false;

	if (pausedFrame_.empty())
		return false;
	pausedFrame_.copyTo(frame);
	pausedDepth_.copyTo(depth);
	cloud = pausedCloud_;
	return true;
}

bool SyncIn::getDepth(Mat &depth, bool pause)
{
	if (!isOpened())
		return false;

	// If not paused, copy the next frame data from
	// frame_. This is the Mat holding the next
	// frame read from the video that update()
	// fills in a separate thread
	// Read if pausedDepth_.empty() to handle
	// case where pause == true on first call
	// to getFrame - without it nothing will ever
	// be read
	if (!pause || pausedDepth_.empty())
		if (!copyBuffers())
			return false;

	if (pausedDepth_.empty())
		return false;
	pausedDepth_.copyTo(depth);
	return true;
}


// Grab point cloud.  If pause is false this copies and latches the
// most recent RGB, depth and point cloud data along with their
// fram number and timestamp.  If pause is true, return the data from
// the most recent call with pause == false.
// The expected use of this function is to have it called with pause = true
// in almost all cases. This way the point cloud data will agree with
// the RGB + D info from the most recent call to getFrame().
// If there's ever a case where we decide not to call getFrame() - maybe
// we don't actually need the RGB + D frames? - then call this with pause == false
bool SyncIn::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause)
{
	if (!isOpened())
		return false;

	usePointCloud_ = true;

	// If not paused, copy the next frame data from
	// frame_. This is the Mat holding the next
	// frame read from the video that update()
	// fills in a separate thread
	// Read if pausedFrame_.empty() to handle
	// case where pause == true on first call
	// to getFrame - without it nothing will ever
	// be read
	if (!pause || pausedFrame_.empty())
		if (!copyBuffers())
			return false;

	cloud = pausedCloud_;
	return true;
}


// Since the update code is running in a 
// different thread, need to mutex lock this since
// it could change the state of the VideoCapture object
// After setting the frame, set frameReady to false
// to force a new frame needs to be read
void SyncIn::frameNumber(int frameNumber)
{
	boost::mutex::scoped_lock guard(mtx_);
	if (postLockFrameNumber(frameNumber))
	{
		setFrameNumber(frameNumber-1);
		frameReady_ = false; // force update to read this frame
		condVar_.notify_all();
	}
}
