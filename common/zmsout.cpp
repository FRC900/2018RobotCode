#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/zlib.hpp>

#include "zmsout.hpp"
#include "cvMatSerialize.hpp"

using namespace std;
using namespace cv;
using namespace boost::filesystem;

// Save the raw camera stream to disk.  This uses a home-brew
// method to serialize image and depth data to disk rather than
// relying on Stereolab's SVO format.
ZMSOut::ZMSOut(const char *outFile, int frameSkip, int splitSize, bool useZlib) :
	MediaOut(frameSkip, splitSize),
	fileName_(outFile),
	serializeOut_(NULL),
	filtSBOut_(NULL),
	archiveOut_(NULL),
	useZlib_(useZlib)
{
	cout << "useZlib = " << useZlib_ << endl;
}

// Clean up pointers, which will also 
// close the files they point to
ZMSOut::~ZMSOut()
{
	deleteOutputPointers();
}

// Serialize the frame plus depth info
bool ZMSOut::write(const Mat &frame, const Mat &depth)
{
	if (!archiveOut_)
		return false;

	*archiveOut_ << frame << depth;
	return true;
}


// Output needs 3 things. First is a standard ofstream to write to
// Next is an (optional) filtered stream buffer. This is used to
// compress on the fly - uncompressed files take up way too
// much space. Last item is the actual boost binary archive template
// If all three are opened, return true. If not, delete and set to
// NULL all pointers related to serialized Output
bool ZMSOut::openSerializeOutput(const char *fileName)
{
	deleteOutputPointers();
	// First create an ofstream to write to
	serializeOut_ = new ofstream(fileName, ios::out | ios::binary);
	if (!serializeOut_ || !serializeOut_->is_open())
	{
		cerr << "Could not open ofstream(" << fileName << ")" << endl;
		deleteOutputPointers();
		return false;
	}
	// Create a filtering streambuf.  Push zlib and then
	// the ofstream created above so output is automatically
	// compressed before writing to disk
	filtSBOut_= new boost::iostreams::filtering_streambuf<boost::iostreams::output>;
	if (!filtSBOut_)
	{
		cerr << "Could not create filtering_streambuf<output> in constructor" <<endl;
		deleteOutputPointers();
		return false;
	}
	if (useZlib_)
		filtSBOut_->push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_speed));
	filtSBOut_->push(*serializeOut_);

	// Create an output archive which writes to the previously
	// created output chain (zlib->output file path)
	archiveOut_ = new portable_binary_oarchive(*filtSBOut_);
	if (!archiveOut_)
	{
		cerr << "Could not create binary_oarchive in constructor" <<endl;
		deleteOutputPointers();
		return false;
	}
	return true;
}

// Create a filename by appening a fileCounter to
// the end of the filename
bool ZMSOut::openNext(int fileCounter)
{
	if (fileName_.length() == 0)
		return false;
	stringstream ofName;
	ofName << change_extension(fileName_, "").string() << "_" << fileCounter << ".zms";
	if (!openSerializeOutput(ofName.str().c_str()))
	{
		cerr << "ZMSOut : could not open output file " << ofName.str() << endl;
		return false;
	}
	return true;
}


// Helper to easily delete and NULLize output file pointers
void ZMSOut::deleteOutputPointers(void)
{
	if (archiveOut_)
	{
		delete archiveOut_;
		archiveOut_ = NULL;
	}
	if (filtSBOut_)
	{
		delete filtSBOut_;
		filtSBOut_ = NULL;
	}
	if (serializeOut_)
	{
		delete serializeOut_;
		serializeOut_ = NULL;
	}
}
