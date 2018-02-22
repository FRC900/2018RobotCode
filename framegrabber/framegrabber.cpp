#include <opencv2/opencv.hpp>

#include <iostream>
using namespace cv;
using namespace std;

// --percent - grab capture every %
// --frames - grab capture every absolute frame #
// --oneFrame - grab a single frame
int main(int argc, char **argv)
{
	const string framesOpt  = "--frames=";
	const string percentOpt = "--percent=";
	const string oneFrameOpt = "--oneFrame=";
	const string badOpt     = "--";
	double percent   = 0.01;
	double frames;
	double startFrame = 0;
	bool   oneFrame = false;
	double framesInc = 0.0;
	int fileArgc;
	for (fileArgc = 1; fileArgc < argc; fileArgc++)
	{
		if (framesOpt.compare(0, framesOpt.length(), argv[fileArgc], framesOpt.length()) == 0)
			framesInc = atoi(argv[fileArgc] + framesOpt.length());
		else if (percentOpt.compare(0, percentOpt.length(), argv[fileArgc], percentOpt.length()) == 0)
			percent = atof(argv[fileArgc] + percentOpt.length())/100.0;
		else if (oneFrameOpt.compare(0, oneFrameOpt.length(), argv[fileArgc], oneFrameOpt.length()) == 0)
		{
			startFrame = atof(argv[fileArgc] + oneFrameOpt.length());
			oneFrame = true;
		}
		else if (badOpt.compare(0, badOpt.length(), argv[fileArgc], badOpt.length()) == 0)
		{
			cerr << "Unknown command line option " << argv[fileArgc] << endl;
			return -1; // unknown option
		}
		else // first non -- arg is filename
			break;
	}

	if (fileArgc >= argc)
	{
		cerr << "Missing file name " << endl;
		return -2;
	}

	VideoCapture cap(argv[fileArgc]);
	string capPath(argv[fileArgc]);
	const size_t last_slash_idx = capPath.find_last_of("\\/");
	if (string::npos != last_slash_idx)
		capPath.erase(0, last_slash_idx + 1);

	frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
	if (framesInc == 0.0)
		framesInc = frames * percent;

	Mat image;
	for (double frame = startFrame; frame < frames; frame += framesInc)
	{
		cap.set(CV_CAP_PROP_POS_FRAMES, frame);

		cap >> image;
		if (!image.empty())
		{
			// Create filename, save image
			stringstream fn;
			fn << capPath;
			fn << "_";
			fn << int(frame);
			fn << ".png";
			imwrite(fn.str(), image);
		}
		if (oneFrame)
			break;
	}
}
