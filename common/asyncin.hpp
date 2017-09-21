// Class to handle asynchronous media inputs. These are inputs
// such as cameras where the input is decoupled from processing. 
// For examples, with cameras we always want the latest frame and
// are OK skipping a few if processing can't keep up with the input
// rate.
// This is a base class which implements a separate thread constantly
// grabbing updates from the camera.  Derived classes will implement
// details of how to get data using a defined interface.
#pragma once

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include "mediain.hpp"

class ZvSettings;

class AsyncIn: public MediaIn
{
	public:
		AsyncIn(ZvSettings *settings = NULL);

		bool getFrame(cv::Mat &frame, bool pause = false);
		bool getFrame(cv::Mat &frame, cv::Mat &depth, bool pause = false);
		bool getFrame(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause = false);
		bool getDepth(cv::Mat &depth, bool pause = true);
		bool getPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool pause = true);

	protected:
		// Derived classes need to start and stop
		// the update thread. startThread should be
		// called once the input source is ready to 
		// send data and stop should be called in the
		// destructor
		void startThread(void);
		void stopThread(void);

		// Defined in derived classes to handle the nuts
		// and bolts of grabbing a frame from a given
		// source.  preLock happens before the mutex
		// while postLock happens inside it
		virtual bool preLockUpdate(void) = 0;
		virtual bool postLockUpdate(cv::Mat &frame, cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB> &cloud) = 0;

	private:
		// Input is buffered several times
		// frame_ is the most recent frame grabbed from 
		// the camera
		// pausedFrame_ is the most recent frame returned
		// from a call to getFrame. If video is paused, this
		// frame is returned multiple times until the
		// GUI is unpaused
		cv::Mat           frame_;
		cv::Mat           depth_;
		cv::Mat           pausedFrame_;
		cv::Mat           pausedDepth_;

		// Same thing for point cloud data
		pcl::PointCloud<pcl::PointXYZRGB> cloud_;
		pcl::PointCloud<pcl::PointXYZRGB> pausedCloud_;

		// Mutex used to protect frame_
		// from simultaneous accesses 
		// by multiple threads
		boost::mutex      mtx_;

		// Thread dedicated to update() loop
		boost::thread thread_;

		// Flag and condition variable to indicate
		// update() has grabbed at least 1 frame
		boost::condition_variable condVar_;
		bool updateStarted_;

		cv::VideoCapture cap_;

		void update(void);
		bool copyBuffers(void);
};
