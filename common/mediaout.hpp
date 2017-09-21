#pragma once

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>

#include "frameticker.hpp"

// Base class for output.  Derived classes are for writing 
// AVI videos, zms (video + depth), plus whatever else we
// imagine in the future.
class MediaOut
{
   public:
		MediaOut(int frameSkip, int framesPerFile);
		virtual ~MediaOut();
		bool saveFrame(const cv::Mat &frame, const cv::Mat &depth);
		void sync(void);
		float FPS(void) const;

   protected:
		// The base class calls these dervied classes to do the 
		// heavy lifting.  They have to be implemented in the 
		// base class as well, but hopefully those are never
		// called 
		virtual bool openNext(int fileCounter);
		virtual bool write(const cv::Mat &frame, const cv::Mat &depth);

   private: 
		// Skip output frames if requested.  Skip is how many to 
		// skip before writing next output frame, FrameCounter is how
		// many total frames seen.
		// Counter is used to split the output into multiple shorter
		// outputs
		int frameSkip_;
		int frameCounter_;
		int fileCounter_;
		int framesPerFile_;
		int framesThisFile_;
		void writeThread(void);
		cv::Mat frame_;
		cv::Mat depth_;
		boost::mutex matLock_;
		boost::condition_variable frameCond_;
		bool frameReady_;
		bool writePending_;
		boost::thread thread_;
		FrameTicker ft_;
};
