#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "pngout.hpp"

using namespace std;
using namespace cv;

// Call the base class constructor plus init some 
// local class members
PNGOut::PNGOut(const char *outFile) :
	MediaOut(1, numeric_limits<int>::max()),
	fileName_(outFile)
{
}

// Delete the writer_ object to close that
// output file
PNGOut::~PNGOut()
{
}

// Write the frame to a file. This might overwrite 
// the last one, but if we're looking at a 
// still image the output should be one as well.
bool PNGOut::write(const Mat &frame, const Mat &depth)
{
	(void)depth;
	return imwrite(fileName_, frame);
}

// Nothing to do here
bool PNGOut::openNext(int fileCounter)
{
	(void)fileCounter;
	return true;
}
