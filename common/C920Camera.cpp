#ifdef __linux__
/*
 * C920Camera.cpp
 *
 * Created on: Dec 31, 2014
 * Author: jrparks
 *
 * Based on OpenCV cap_v4l.cpp
 */
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "C920Camera.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))

namespace v4l2 {

	static int CAPTURE_SIZE_WIDTHS[] = { 160, 160, 176, 320, 320, 352, 432, 640, 640, 800, 800, 864, 960, 1024, 1280, 1600, 1920 };
	static int CAPTURE_SIZE_HEIGHTS[] = { 90, 120, 144, 180, 240, 288, 240, 360, 480, 448, 600, 480, 720, 576, 720, 896, 1080 };
	static int CAPTURE_FPS_NUMERATOR[] = { 1, 1, 1, 1, 1, 2, 1 };
	static int CAPTURE_FPS_DENOMINATOR[] = { 30, 24, 20, 15, 10, 15, 5 };

	void GetCaptureSize(enum CaptureSize size, unsigned int &width, unsigned int &height) 
	{
		width = CAPTURE_SIZE_WIDTHS[size];
		height = CAPTURE_SIZE_HEIGHTS[size];
	}
	void GetCaptureFPS(enum CaptureFPS fps, unsigned int &numerator, unsigned int &denominator)
	{
		numerator = CAPTURE_FPS_NUMERATOR[fps];
		denominator = CAPTURE_FPS_DENOMINATOR[fps];
	}
	static int xioctl(int fd, int request, void *arg) {
		int r;
		do {
			r = ioctl(fd, request, arg);
		} while (-1 == r && EINTR == errno);
		return r;
	}
	C920Camera::C920Camera() :
		capture(NULL)
	{
	}
	C920Camera::C920Camera(const char* __capture_file) :
		capture(NULL)
	{
		this->Open(__capture_file);
	}
	C920Camera::C920Camera(const int __capture_id) :
		capture(NULL)
	{
		char __capture_file[PATH_MAX];
		sprintf(__capture_file, "/dev/video%1d", __capture_id);
		this->Open(__capture_file);
	}
	C920Camera::~C920Camera() {
		this->Close();
	}
	int C920Camera::Open(const char *__capture_file) {
		fprintf(stdout, "V4L2Camera INFO: Opening capture device %s.\n", __capture_file);
		this->Close();
		this->capture = this->CreateCapture(__capture_file);
		if (this->capture)
			fprintf(stdout, "V4L2Camera INFO: Opened capture device %s.\n", this->capture->DeviceName);
		return this->capture != 0;
	}
	void C920Camera::Close() {
		if (this->capture != 0) {
			this->CloseCapture(this->capture);
			cvFree(&capture);
			this->capture = NULL;
		}
	}
	bool C920Camera::IsOpen() const {
		return this->capture != 0;
	}
	bool C920Camera::GrabFrame() {
		return this->GrabFrame(this->capture);
	}
	IplImage* C920Camera::RetrieveFrame() {
		return this->RetrieveFrame(this->capture);
	}
	bool C920Camera::RetrieveMat(cv::Mat &image) {
		IplImage* _img = this->RetrieveFrame(this->capture);
		if (!_img) {
			image.release();
			return false;
		}
#if CV_MAJOR_VERSION == 2
		cv::Mat temp(_img);
#elif CV_MAJOR_VERSION == 3
		cv::Mat temp = cv::cvarrToMat(_img);
#endif
		if (_img->origin == IPL_ORIGIN_TL)
			temp.copyTo(image);
		else 
			cv::flip(temp, image, 0);
		return true;
	}
	bool C920Camera::ChangeCaptureSize(enum CaptureSize cameracapturesize) {
		return this->ChangeCaptureSizeAndFPS(cameracapturesize, this->capture->CameraCaptureFPS);
	}
	bool C920Camera::ChangeCaptureFPS(enum CaptureFPS cameracapturefps) {
		return this->ChangeCaptureSizeAndFPS(this->capture->CameraCaptureSize, cameracapturefps);
	}
	bool C920Camera::ChangeCaptureSizeAndFPS(enum CaptureSize cameracapturesize, enum CaptureFPS cameracapturefps) {
		this->capture->CameraCaptureSize = cameracapturesize;
		this->capture->CameraCaptureFPS = cameracapturefps;
		fprintf(stdout, "V4L2Camera INFO: Changing capture image size for %s.\n", capture->DeviceName);
		char* __capture_file;
		__capture_file = strdup(this->capture->DeviceName);
		this->CloseCapture(capture);
		this->capture->DeviceName = strdup(__capture_file);
		return this->InitializeCapture(this->capture);
	}
	/* Setters for camera properties */
	bool C920Camera::SetBrightness(int value) {
		this->capture->V4L2Control.id = V4L2_CID_BRIGHTNESS;
		this->capture->V4L2Control.value = value;
		return this->SetControl(this->capture);
	}
	bool C920Camera::SetContrast(int value) {
		this->capture->V4L2Control.id = V4L2_CID_CONTRAST;
		this->capture->V4L2Control.value = value;
		return this->SetControl(this->capture);
	}
	bool C920Camera::SetSaturation(int value) {
		this->capture->V4L2Control.id = V4L2_CID_SATURATION;
		this->capture->V4L2Control.value = value;
		return this->SetControl(this->capture);
	}
	bool C920Camera::SetSharpness(int value) {
		this->capture->V4L2Control.id = V4L2_CID_SHARPNESS;
		this->capture->V4L2Control.value = value;
		return this->SetControl(this->capture);
	}
	bool C920Camera::SetGain(int value) {
		this->capture->V4L2Control.id = V4L2_CID_GAIN;
		this->capture->V4L2Control.value = value;
		return this->SetControl(this->capture);
	}
	bool C920Camera::SetBacklightCompensation(int value) {
		this->capture->V4L2Control.id = V4L2_CID_BACKLIGHT_COMPENSATION;
		this->capture->V4L2Control.value = value;
		return this->SetControl(this->capture);
	}
	bool C920Camera::SetAutoExposure(int value)
	{
		this->capture->V4L2Control.id = V4L2_CID_EXPOSURE_AUTO;
		this->capture->V4L2Control.value = value;
		return this->SetControl(this->capture);
	}
	bool C920Camera::SetFocus(int value) {
		if (value < 0) {
			this->capture->V4L2Control.id = V4L2_CID_FOCUS_AUTO;
			this->capture->V4L2Control.value = true;
			return this->SetControl(this->capture);
		} 
		this->capture->V4L2Control.id = V4L2_CID_FOCUS_AUTO;
		this->capture->V4L2Control.value = false;
		if (this->SetControl(this->capture)) {
			this->capture->V4L2Control.id = V4L2_CID_FOCUS_ABSOLUTE;
			this->capture->V4L2Control.value = value;
			return this->SetControl(this->capture);
		}
		return false;
	}
	bool C920Camera::SetWhiteBalanceTemperature(int value) {
		if (value < 0) {
			this->capture->V4L2Control.id = V4L2_CID_AUTO_WHITE_BALANCE;
			this->capture->V4L2Control.value = true;
			return this->SetControl(this->capture);
		} 
		this->capture->V4L2Control.id = V4L2_CID_AUTO_WHITE_BALANCE;
		this->capture->V4L2Control.value = false;
		if (this->SetControl(this->capture)) {
			this->capture->V4L2Control.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
			this->capture->V4L2Control.value = value;
			return this->SetControl(this->capture);
		}
		return false;
	}
	/* Getters for camera properties */
	bool C920Camera::GetBrightness(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_BRIGHTNESS;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	bool C920Camera::GetContrast(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_CONTRAST;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	bool C920Camera::GetSaturation(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_SATURATION;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	bool C920Camera::GetSharpness(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_SHARPNESS;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	bool C920Camera::GetGain(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_GAIN;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	bool C920Camera::GetBacklightCompensation(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_BACKLIGHT_COMPENSATION;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	bool C920Camera::GetFocus(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_FOCUS_AUTO;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		if (value) {
			value = -1;
			return true;
		}
		this->capture->V4L2Control.id = V4L2_CID_FOCUS_ABSOLUTE;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	bool C920Camera::GetWhiteBalanceTemperature(int &value) {
		this->capture->V4L2Control.id = V4L2_CID_AUTO_WHITE_BALANCE;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		if (value) {
			value = -1;
			return true;
		}
		this->capture->V4L2Control.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
		if (!this->GetControl(this->capture))
			return false;
		value = this->capture->V4L2Control.value;
		return true;
	}
	/* Private Region*/
	void C920Camera::CloseCapture(V4L2CameraCapture* capture) {
		if (capture) {
			fprintf(stdout, "V4L2Camera INFO: Stopping capture stream %s.\n", capture->DeviceName);
			capture->V4L2BufferType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (xioctl(capture->DeviceHandle, VIDIOC_STREAMOFF, &capture->V4L2BufferType) < 0) {
				perror("Unable to stop the stream.");
			}
			fprintf(stdout, "V4L2Camera INFO: Closing capture buffers %s.\n", capture->DeviceName);
			for (unsigned int n_buffers = 0; n_buffers < capture->V4L2RequestBuffers.count; ++n_buffers) {
				if (-1 == munmap(capture->Buffers[n_buffers].start, capture->Buffers[n_buffers].length))
					perror("munmap");
			}
			fprintf(stdout, "V4L2Camera INFO: Closing capture %s.\n", capture->DeviceName);
			if (capture->DeviceHandle != -1) {
				close(capture->DeviceHandle);
			}
			fprintf(stdout, "V4L2Camera INFO: Closing capture frame %s.\n", capture->DeviceName);
			if (capture->Frame.imageData)
				cvFree(&capture->Frame.imageData);
			fprintf(stdout, "V4L2Camera INFO: Closing capture buffer %s.\n", capture->DeviceName);
			if (capture->Buffers[MAX_V4L_BUFFERS].start) {
				free(capture->Buffers[MAX_V4L_BUFFERS].start);
				capture->Buffers[MAX_V4L_BUFFERS].start = NULL;
			}
			free(capture->DeviceName);
			capture->DeviceName = NULL;
		}
	}
	V4L2CameraCapture* C920Camera::CreateCapture(const char *__capture_file) {
		fprintf(stdout, "C920Camera::CreateCapture INFO: Creating capture device %s.\n", __capture_file);
		V4L2CameraCapture* capture = (V4L2CameraCapture*) cvAlloc(sizeof(V4L2CameraCapture));
		if (!capture) {
			fprintf(stderr, "C920Camera::CreateCapture ERROR: V4L: Could not allocate memory for capture process.\n");
			return NULL;
		}
		memset(capture, 0, sizeof(V4L2CameraCapture));
		capture->DeviceName = strdup(__capture_file);
		// Set Defaults
		capture->CameraCaptureSize = DEFAULT_CAPTURE_SIZE;
		capture->CameraCaptureFPS = DEFAULT_CAPTURE_FPS;
		fprintf(stdout, "C920Camera::CreateCapture INFO: V4L: Initializing capture device %s.\n", capture->DeviceName);
		if (this->InitializeCapture(capture) != true) {
			fprintf(stderr, "C920Camera::CreateCapture ERROR: V4L: Could not initialize capture %s.\n", capture->DeviceName);
			return NULL;
		}
		fprintf(stdout, "C920Camera::CreateCapture INFO: Created capture device %s.\n", capture->DeviceName);
		return capture;
	}
	int C920Camera::InitializeCapture(V4L2CameraCapture* capture) {
		capture->Buffers[MAX_V4L_BUFFERS].start = NULL;
		fprintf(stdout, "C920Camera::InitilizeCapture INFO: Opening capture device %s.\n", capture->DeviceName);
		capture->DeviceHandle = open(capture->DeviceName, O_RDWR /* required */| O_NONBLOCK, 0);
		if (capture->DeviceHandle == 0) {
			fprintf(stderr, "C920Camera::InitilizeCapture ERROR: Unable to open %s.\n", capture->DeviceName);
			this->CloseCapture(capture);
			return -1;
		}
		fprintf(stdout, "C920Camera::InitilizeCapture INFO: Quering capture device capability %s.\n", capture->DeviceName);
		CLEAR(capture->V4L2Capability);
		if (-1 == xioctl(capture->DeviceHandle, VIDIOC_QUERYCAP, &capture->V4L2Capability)) {
			fprintf(stderr, "C920Camera::InitilizeCapture ERROR: Unable to query capability from %s.\n", capture->DeviceName);
			this->CloseCapture(capture);
			return -2;
		}
		if ((capture->V4L2Capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
			fprintf(stderr, "C920Camera::InitilizeCapture ERROR: V4L2: Device %s is unable to capture video memory.\n",
					capture->DeviceName);
			this->CloseCapture(capture);
			return -3;
		}
		if (!this->SetCaptureFormat(capture)) {
			fprintf(stderr, "C920Camera::InitilizeCapture ERROR: Unable to set capture format.\n");
			return -4;
		}
		if (this->InitializeCaptureBuffers(capture) != true) {
			fprintf(stderr, "C920Camera::InitilizeCapture ERROR: Unable to initialize capture buffers.\n");
			return -5;
		}
		/* Set up Image data */
		cvInitImageHeader(&capture->Frame, cvSize(capture->V4L2Format.fmt.pix.width, capture->V4L2Format.fmt.pix.height),
				IPL_DEPTH_8U, 3, IPL_ORIGIN_TL, IPL_ALIGN_4BYTES);
		/* Allocate space for RGBA data */
		capture->Frame.imageData = (char *) cvAlloc(capture->Frame.imageSize);
		capture->NeedsCaptureInitialization = true;
		return true;
	}
	int C920Camera::SetCaptureFormat(V4L2CameraCapture* capture) {
		fprintf(stdout, "C920Camera::SetCaptureFormat INFO: Setting capture device format %s.\n", capture->DeviceName);
		CLEAR(capture->V4L2Format);
		capture->V4L2Format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		capture->V4L2Format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
		capture->V4L2Format.fmt.pix.field = V4L2_FIELD_NONE;
		//capture->V4L2Format.fmt.pix.width = capture->CaptureWidth;
		//capture->V4L2Format.fmt.pix.height = capture->CaptureHeight;
		GetCaptureSize(capture->CameraCaptureSize, capture->V4L2Format.fmt.pix.width, capture->V4L2Format.fmt.pix.height);
		if (-1 == xioctl(capture->DeviceHandle, VIDIOC_S_FMT, &capture->V4L2Format)) {
			fprintf(stderr, "C920Camera::SetCaptureFormat ERROR: Unable to xioctl VIDIOC_S_FMT.\n");
			return false;
		}
		capture->V4L2StreamParmeters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		//capture->V4L2StreamParmeters.parm.capture.timeperframe.numerator = 1;
		//capture->V4L2StreamParmeters.parm.capture.timeperframe.denominator = 30;
		GetCaptureFPS(capture->CameraCaptureFPS, capture->V4L2StreamParmeters.parm.capture.timeperframe.numerator,
				capture->V4L2StreamParmeters.parm.capture.timeperframe.denominator);
		if (-1 == xioctl(capture->DeviceHandle, VIDIOC_S_PARM, &capture->V4L2StreamParmeters)) {
			fprintf(stderr, "C920Camera::SetCaptureFormat ERROR: Unable to xioctl VIDIOC_S_PARM.\n");
			return false;
		}
		return true;
	}
	int C920Camera::InitializeCaptureBuffers(V4L2CameraCapture* capture) {
		fprintf(stdout, "C920Camera::InitializeCaptureBuffers INFO: Initializing capture device buffers %s.\n",
				capture->DeviceName);
		CLEAR(capture->V4L2RequestBuffers);
		unsigned int buffer_number = DEFAULT_V4L_BUFFERS;
try_again: capture->V4L2RequestBuffers.count = buffer_number;
		   capture->V4L2RequestBuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		   capture->V4L2RequestBuffers.memory = V4L2_MEMORY_MMAP;
		   if (-1 == xioctl(capture->DeviceHandle, VIDIOC_REQBUFS, &capture->V4L2RequestBuffers)) {
			   if (EINVAL == errno) {
				   fprintf(stderr, "C920Camera::InitializeCaptureBuffers ERROR: %s does not support memory mapping\n",
						   capture->DeviceName);
			   } else {
				   perror("VIDIOC_REQBUFS");
			   }
			   /* free capture, and returns an error code */
			   this->CloseCapture(capture);
			   return -1;
		   }
		   if (capture->V4L2RequestBuffers.count < buffer_number) {
			   if (buffer_number == 1) {
				   fprintf(stderr, "C920Camera::InitializeCaptureBuffers ERROR: Insufficient buffer memory on %s\n",
						   capture->DeviceName);
				   /* free capture, and returns an error code */
				   this->CloseCapture(capture);
				   return -2;
			   } 
			   buffer_number--;
			   fprintf(stderr,
					   "C920Camera::InitializeCaptureBuffers ERROR: Insufficient buffer memory on %s -- decreaseing buffers\n",
					   capture->DeviceName);
			   goto try_again;
		   }
		   for (unsigned int n_buffers = 0; n_buffers < capture->V4L2RequestBuffers.count; ++n_buffers) {
			   struct v4l2_buffer buffer;
			   CLEAR(buffer);
			   buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			   buffer.memory = V4L2_MEMORY_MMAP;
			   buffer.index = n_buffers;
			   if (-1 == xioctl(capture->DeviceHandle, VIDIOC_QUERYBUF, &buffer)) {
				   perror("VIDIOC_QUERYBUF");
				   /* free capture, and returns an error code */
				   this->CloseCapture(capture);
				   return -3;
			   }
			   capture->Buffers[n_buffers].length = buffer.length;
			   capture->Buffers[n_buffers].start = mmap(NULL /* start anywhere */, buffer.length,
					   PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, capture->DeviceHandle, buffer.m.offset);
			   if (MAP_FAILED == capture->Buffers[n_buffers].start) {
				   perror("mmap");
				   /* free capture, and returns an error code */
				   this->CloseCapture(capture);
				   return -4;
			   }
			   if (n_buffers == 0) {
				   if (capture->Buffers[MAX_V4L_BUFFERS].start) {
					   free(capture->Buffers[MAX_V4L_BUFFERS].start);
					   capture->Buffers[MAX_V4L_BUFFERS].start = NULL;
				   }
				   capture->Buffers[MAX_V4L_BUFFERS].start = malloc(buffer.length);
				   capture->Buffers[MAX_V4L_BUFFERS].length = buffer.length;
			   };
		   }
		   return true;
	}
	bool C920Camera::GrabFrame(V4L2CameraCapture* capture) {
		if (capture->NeedsCaptureInitialization) {
			fprintf(stdout, "C920Camera::GrabFrame INFO: Querying capture device buffers %s.\n", capture->DeviceName);
			for (capture->BufferIndex = 0; capture->BufferIndex < ((int) capture->V4L2RequestBuffers.count);
					++capture->BufferIndex) {
				struct v4l2_buffer buf;
				CLEAR(buf);
				buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory = V4L2_MEMORY_MMAP;
				buf.index = (unsigned long) capture->BufferIndex;
				if (-1 == xioctl(capture->DeviceHandle, VIDIOC_QBUF, &buf)) {
					perror("VIDIOC_QBUF");
					return false;
				}
			}
			fprintf(stdout, "C920Camera::GrabFrame INFO: Starting capture device stream %s.\n", capture->DeviceName);
			/* enable the streaming */
			capture->V4L2BufferType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (-1 == xioctl(capture->DeviceHandle, VIDIOC_STREAMON, &capture->V4L2BufferType)) {
				/* error enabling the stream */
				perror("VIDIOC_STREAMON");
				return false;
			}
			// Skip first Frame from camera.
			this->V4L2Loop(capture);
			capture->NeedsCaptureInitialization = false;
		}
		// Read Frame from camera.
		this->V4L2Loop(capture);
		return true;
	}
	void C920Camera::V4L2Loop(V4L2CameraCapture* capture) {
		while (true) {
			fd_set fds;
			struct timeval tv;
			int r;
			FD_ZERO(&fds);
			FD_SET(capture->DeviceHandle, &fds);
			/* Timeout. */
			tv.tv_sec = 10;
			tv.tv_usec = 0;
			r = select(capture->DeviceHandle + 1, &fds, NULL, NULL, &tv);
			if (-1 == r) {
				if (EINTR == errno)
					continue;
				perror("select");
			}
			if (0 == r) {
				fprintf(stderr, "select timeout\n");
				/* end the infinite loop */
				break;
			}
			if (this->ReadFrame(capture))
				break;
		}
	}
	int C920Camera::ReadFrame(V4L2CameraCapture* capture) {
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if (-1 == xioctl(capture->DeviceHandle, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
				case EAGAIN:
					return 0;
				case EIO:
					/* Could ignore EIO, see spec. */
					/* fall through */
				default:
					/* display the error and stop processing */
					perror("VIDIOC_DQBUF");
					return 1;
			}
		}
		assert(buf.index < capture->V4L2RequestBuffers.count);
		memcpy(capture->Buffers[MAX_V4L_BUFFERS].start, capture->Buffers[buf.index].start,
				capture->Buffers[MAX_V4L_BUFFERS].length);
		capture->BufferIndex = MAX_V4L_BUFFERS;
		// printf("got data in buff %d, len=%d, flags=0x%X, seq=%d, used=%d)\n", buf.index, buf.length, buf.flags, buf.sequence,
		// buf.bytesused);
		if (-1 == xioctl(capture->DeviceHandle, VIDIOC_QBUF, &buf))
			perror("VIDIOC_QBUF");
		return 1;
	}
	IplImage* C920Camera::RetrieveFrame(V4L2CameraCapture* capture) {
		// Resize Frame if Format size has changed.
		if (((unsigned long) capture->Frame.width != capture->V4L2Format.fmt.pix.width)
				|| ((unsigned long) capture->Frame.height != capture->V4L2Format.fmt.pix.height)) {
			cvFree(&capture->Frame.imageData);
			cvInitImageHeader(&capture->Frame, cvSize(capture->V4L2Format.fmt.pix.width, capture->V4L2Format.fmt.pix.height),
					IPL_DEPTH_8U, 3, IPL_ORIGIN_TL, 4);
			capture->Frame.imageData = (char *) cvAlloc(capture->Frame.imageSize);
		}
		// Decode image from MJPEG to RGB24
		if ((capture->Buffers[capture->BufferIndex].start) &&
			!this->MJPEG2RGB24(capture->V4L2Format.fmt.pix.width, 
				               capture->V4L2Format.fmt.pix.height,
					           (unsigned char*) capture->Buffers[capture->BufferIndex].start, 
							   capture->Buffers[capture->BufferIndex].length,
					           (unsigned char*) capture->Frame.imageData)) {
			fprintf(stdout, "C920Camera::RetrieveFrame ERROR: Unable to decode frame.\n");
			return 0;
		}
		return &capture->Frame;
	}
	bool C920Camera::MJPEG2RGB24(int width, int height, unsigned char *src, int length, unsigned char *dst) {
		cv::Mat temp = cv::imdecode(cv::Mat(std::vector<uchar>(src, src + length)), 1);
		if (!temp.data || temp.cols != width || temp.rows != height)
			return false;
		memcpy(dst, temp.data, width * height * 3);
		return true;
	}
	int C920Camera::SetControl(V4L2CameraCapture* capture) {
		if (xioctl(capture->DeviceHandle, VIDIOC_S_CTRL, &capture->V4L2Control) == -1) {
			fprintf(stderr, "C920Camera::SetControl ERROR: Unable to set control...\n");
			return false;
		}
		return true;
	}
	int C920Camera::GetControl(V4L2CameraCapture* capture) {
		if (xioctl(capture->DeviceHandle, VIDIOC_G_CTRL, &capture->V4L2Control) == -1) {
			fprintf(stderr, "C920Camera::GetControl ERROR: Unable to get control...\n");
			return false;
		}
		return true;
	}
} /* namespace v4l2 */
#endif
