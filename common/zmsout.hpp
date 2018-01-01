#pragma once
#include "portable_binary_oarchive.hpp"
#include <boost/iostreams/filtering_streambuf.hpp>
#include "mediaout.hpp"

// Hack up a way to save zed data - serialize both
// BGR frame and depth frame
class ZMSOut : public MediaOut
{
	public:
		ZMSOut(const char *outFile, int frameSkip = 0, int splitSize = 150, bool useZlib = true);
		~ZMSOut();

		// Make non-copyable
		ZMSOut(const ZMSOut &zmsout) = delete;
		ZMSOut &operator=(const ZMSOut &zmsout) = delete;
	private :
		bool openNext(int fileCounter) override;
		bool write(const cv::Mat &frame, const cv::Mat &depth) override;
		void deleteOutputPointers(void);
		bool openSerializeOutput(const char *filename);

		std::string fileName_;

		std::ofstream *serializeOut_;
		boost::iostreams::filtering_streambuf<boost::iostreams::output> *filtSBOut_;
		portable_binary_oarchive *archiveOut_;
		bool useZlib_;
};
