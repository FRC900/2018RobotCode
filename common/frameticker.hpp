#ifndef INC_FRAMETICKER_HPP__
#define INC_FRAMETICKER_HPP__

#include <opencv2/opencv.hpp>
#include <boost/circular_buffer.hpp>

class FrameTicker
{
	public :
		FrameTicker() :
			frameTicks_{5},
			start_(0),
			divider_(cv::getTickFrequency())
		{
		}

		// Save an elapsed time since last call
		// to mark, re-init start_ to right now
		void mark(void)
		{
			if (start_)
			{
				double end = cv::getTickCount();
				frameTicks_.push_back((end - start_) / divider_); 
			}
			start_ = cv::getTickCount();
		}

		bool valid(void) const
		{
			return frameTicks_.full();
		}

		double getFPS(void) const
		{
			if (!valid())
				return -1;
			double sum = 0.0;
			for (auto it = frameTicks_.begin(); it != frameTicks_.end(); ++it)
				sum += *it;
			return frameTicks_.size() / sum;
		}
	private :
		boost::circular_buffer<double> frameTicks_;
		int64   start_;
		double  divider_;
};

#endif

