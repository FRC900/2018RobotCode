#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>

#include "aviout.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

// Call the base class constructor plus init some 
// local class members
AVIOut::AVIOut(const char *outFile, const Size &size, int frameSkip):
	MediaOut(frameSkip, 900),
	size_(size),
	writer_(NULL),
	fileName_(outFile)
{
}

// Delete the writer_ object to close that
// output file
AVIOut::~AVIOut()
{
	if (writer_)
		delete writer_;
}

// Make sure the writer is initialized. If so,
// write the next frame to it.
bool AVIOut::write(const Mat &frame, const Mat &depth)
{
	(void)depth;
	if (!writer_ || !writer_->isOpened())
		return false;

	*writer_ << frame;
	return true;
}

// Open the next file in the sequence.  The code is
// set by default to split outputs into multiple 
// files. This way if we have corruption on one 
// because we powered off we don't lose all of them.
bool AVIOut::openNext(int fileCounter)
{
	if (fileName_.length() == 0)
		return false;

	if (writer_)
	{
		delete writer_;
		writer_ = NULL;
	}
	stringstream ofName;
	ofName << change_extension(fileName_, "").string() << "_" << fileCounter << ".avi";
	writer_ = new VideoWriter(ofName.str(), CV_FOURCC('M','J','P','G'), 30, size_, true);
	if(!writer_ || !writer_->isOpened())
	{
		std::cerr << "AVIOut() : Could not open output video " << ofName.str() << std::endl;
		delete writer_;
		writer_ = NULL;
		return false;
	}
	return true;
}
