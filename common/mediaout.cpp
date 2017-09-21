#include <iostream>
#include "mediaout.hpp"
#include "frameticker.hpp"

using namespace cv;

// Set up variables to skip frames and split
// between files as set up by the derived class
// Initialize frameReady shared var to false and 
// kick off the writer thread
MediaOut::MediaOut(int frameSkip, int framesPerFile) :
	frameSkip_(max(frameSkip, 1)),
	frameCounter_(0),
	fileCounter_(0),
	framesPerFile_(framesPerFile),
	framesThisFile_(framesPerFile_),
	frameReady_(false),
	writePending_(false),
	thread_(boost::bind(&MediaOut::writeThread, this))
{
}

MediaOut::~MediaOut(void)
{
	// Make sure any pending frames have been
	// written, then shut down the writer
	// thread.  Once that's finished, exit
	sync();
  	thread_.interrupt();
  	thread_.join();
}

// Save a frame if there have been frameSkip_ frames
// since the last write (frameSkip == 1 writes every frame,
// == 2 every other, and so on).
// Open a new output file if framesPerFile_ frames have been written
// to the current video
bool MediaOut::saveFrame(const Mat &frame, const Mat &depth)
{
	// Every frameSkip_ frames, copy another frame
	// to the frame_ and depth_ vars. Then set frameReady_
	// to trigger the writer thread to grab them and
	// write them to disk
	if ((frameCounter_++ % frameSkip_) == 0)
	{
		boost::mutex::scoped_lock lock(matLock_);

		// Open a new video when we've written framesThisFile
		// framesThisFile is initialized to framesPerFile so
		// this also opens the file the first time this
		// method is called
		if (framesThisFile_ >= framesPerFile_)
		{
			// Wait until pending writes are complete
			// before closing the previous file
			while (frameReady_ || writePending_)
				frameCond_.wait(lock);

			if (!openNext(fileCounter_++))
				return false;
			framesThisFile_ = 0;
		}
		// Copy input args to shared frame_ and depth_
		// buffers. Do it unconditionally. This means that
		// it is possible for the update thread to miss
		// frames if two calls to this function come in back 
		// to back. That's OK - the goal here is to
		// write as many frames as possible
		// without slowing down performance rather
		// that waiting to be sure that every frame
		// it captured to disk.  Programs which
		// need to save every frame can explicitly
		// call the sync() method below before
		// each call to this function.
		frame.copyTo(frame_);
		depth.copyTo(depth_);
		frameReady_ = true;
		// Set a flag to indicate there is a disk write
		// that needs to complete
		writePending_ = true;

		// Notifiy other threads waiting
		// on the mutex that it is now
		// available - wake them up to do work
		frameCond_.notify_all();
	}

	// If we made it this far, the write was successful.
	// Note that it might be that nothing was written if
	// this frame was skipped, but that's not an error
	return true;
}

// Dummy member functions - base class shouldn't be called
// directly so these shouldn't be used
bool MediaOut::openNext(int fileCounter)
{
	(void)fileCounter;
	return false;
}

bool MediaOut::write(const Mat &frame, const Mat &depth)
{
	(void)frame;
	(void)depth;
	return false;
}

// Separate thread to write video frames to disk
// This runs as quickly as possible, but is also
// designed to drop frames if it gets behind the
// main processing thread.  
void MediaOut::writeThread(void)
{
	Mat frame;
	Mat depth;
	while (true)
	{
		// Grab the lock mutex
		// Check that frameReady is set
		// if it hasn't, that means there's
		// no new data to write.  In that case,
		// call wait() to release the mutex and 
		// loop around to try again. This lets a
		// call to saveFrame to complete if one
		// is waiting on the mutex.
		// Once frameReady_ has been set, copy
		// the data out of the member variables
		// into a local var, release the lock, 
		// and do the write with the copies
		// of the Mats.  Using the copies will
		// let saveFrame write new data to the shared vars
		// while write() works on the old frame.
		// Note that saveFrame intentionally
		// doesn't check to see if frame_ and
		// depth_ have been copied by this thread
		// before writing to them.
		// This way if the write() call takes too
		// long it is possible for saveFrame to
		// update the frame_ and depth_ vars more 
		// than once before this thread reads them again.
		// That will potentially drop frames, but it
		// also lets the main thread run as quickly
		// as possible rather than waiting on this thread
		{
			boost::mutex::scoped_lock lock(matLock_);
			while (!frameReady_)
				frameCond_.wait(lock);

			frame_.copyTo(frame);
			depth_.copyTo(depth);
			frameReady_ = false;
		}

		// Call a derived class' write() method
		// to actually format and write the data to disk
		write(frame, depth);
		
		// If there's no frame in the buffer above
		// clear out writePending_
		// Can't just unconditionally clear it since
		// it is likely that saveFrame() added a new
		// frame to the shared buffer while the 
		// write() call just above was taking place
		{
			boost::mutex::scoped_lock lock(matLock_);
			// Update count of frames actually
			// written to the output
			framesThisFile_ += 1;
			if (!frameReady_)
			{
				writePending_ = false;
				frameCond_.notify_all();
			}
		}
		ft_.mark();

		boost::this_thread::interruption_point();
	}
}

float MediaOut::FPS(void) const
{
	return ft_.getFPS();
}

// Loop until any pending write has completed
// Normally we don't care if the writer gets out of sync
// and drops frames, so long as the main thread isn't
// slowed down in the process.  In some caes, though,
// we do want to make sure everything gets written.
// 1 - when converting or marking up a video input, we want
//     to make sure every input frame is processed into
//     the output video
// 2 - when auto-splitting videos every N frames, we need
//     to make sure the last frame is written before
//     closing the old file and moving on to the new one
// 3 - when shutting down, be sure all writes are finished
//     before closing the output files
void MediaOut::sync(void)
{
	boost::mutex::scoped_lock lock(matLock_);
	while (frameReady_ || writePending_)
		frameCond_.wait(lock);
}
