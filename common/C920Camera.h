/*
 * C920Camera.h
 *
 * Created on: Dec 31, 2014
 * Author: jrparks
 *
 * Based on OpenCV cap_v4l.cpp
 */
#ifndef C920CAMERA_H_
#define C920CAMERA_H_
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <asm/types.h> /* for videodev2.h */
#include <assert.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#define DEFAULT_CAPTURE_SIZE CAPTURE_SIZE_320x240
#define DEFAULT_CAPTURE_FPS CAPTURE_FPS_30
#define MAX_V4L_BUFFERS 4
#define DEFAULT_V4L_BUFFERS 4
namespace v4l2 {
   enum CaptureSize {
      CAPTURE_SIZE_160x90 = 0, // Sizes
      CAPTURE_SIZE_160x120, // Normal
      CAPTURE_SIZE_176x144,
      CAPTURE_SIZE_320x180,
      CAPTURE_SIZE_320x240, // Normal
      CAPTURE_SIZE_352x288,
      CAPTURE_SIZE_432x240,
      CAPTURE_SIZE_640x360,
      CAPTURE_SIZE_640x480, // Normal
      CAPTURE_SIZE_800x448,
      CAPTURE_SIZE_800x600,
      CAPTURE_SIZE_864x480,
      CAPTURE_SIZE_960x720,
      CAPTURE_SIZE_1024x576,
      CAPTURE_SIZE_1280x720, // Normal
      CAPTURE_SIZE_1600x896,
      CAPTURE_SIZE_1920x1080, // Normal
   };
   void GetCaptureSize(enum CaptureSize size, unsigned int &width, unsigned int &height);

   enum CaptureFPS {
      CAPTURE_FPS_30 = 0, // FPS
      CAPTURE_FPS_24,
      CAPTURE_FPS_20,
      CAPTURE_FPS_15,
      CAPTURE_FPS_10,
      CAPTURE_FPS_7_5,
      CAPTURE_FPS_5,
   };
   void GetCaptureFPS(enum CaptureFPS fps, unsigned int &numerator, unsigned int &denominator);

   struct V4L2Buffer {
      void * start;
      size_t length;
   };
   struct V4L2CameraCapture {
      char* DeviceName;
      int DeviceHandle;
      int NeedsCaptureInitialization;
      enum CaptureSize CameraCaptureSize;
      enum CaptureFPS CameraCaptureFPS;
      IplImage Frame;
      int BufferIndex;
      V4L2Buffer Buffers[DEFAULT_V4L_BUFFERS + 1];
      /* V4L2 Structs */
      struct v4l2_capability V4L2Capability;
      struct v4l2_format V4L2Format;
      struct v4l2_streamparm V4L2StreamParmeters;
      struct v4l2_control V4L2Control;
      struct v4l2_queryctrl V4L2QueryControl;
      struct v4l2_requestbuffers V4L2RequestBuffers;
      enum v4l2_buf_type V4L2BufferType;
   };
   class C920Camera {
      public:
         C920Camera();
         C920Camera(const char *__capture_file);
         C920Camera(const int __capture_id);
		 C920Camera(const C920Camera &c920camera) = delete;
		 C920Camera& operator=(const C920Camera &c920camera) = delete;
         virtual ~C920Camera();
         int Open(const char *__capture_file);
         void Close();
         bool IsOpen() const;
         bool GrabFrame();
         IplImage* RetrieveFrame();
         bool RetrieveMat(cv::Mat &frame);
         bool ChangeCaptureSizeAndFPS(enum CaptureSize cameracapturesize, enum CaptureFPS cameracapturefps);
         bool ChangeCaptureSize(enum CaptureSize cameracapturesize);
         bool ChangeCaptureFPS(enum CaptureFPS cameracapturefps);
         bool SetBrightness(int value);
         bool SetContrast(int value);
         bool SetSaturation(int value);
         bool SetSharpness(int value);
         bool SetGain(int value);
         bool SetBacklightCompensation(int value);
         bool SetAutoExposure(int value);
         bool SetFocus(int value);
         bool SetWhiteBalanceTemperature(int value);
         bool GetBrightness(int &value);
         bool GetContrast(int &value);
         bool GetSaturation(int &value);
         bool GetSharpness(int &value);
         bool GetGain(int &value);
         bool GetBacklightCompensation(int &value);
         bool GetFocus(int &value);
         bool GetWhiteBalanceTemperature(int &value);
      protected:
         V4L2CameraCapture* capture;
         void CloseCapture(V4L2CameraCapture* capture);
         V4L2CameraCapture* CreateCapture(const char *__capture_file);
         int InitializeCapture(V4L2CameraCapture* capture);
         int SetCaptureFormat(V4L2CameraCapture* capture);
         int InitializeCaptureBuffers(V4L2CameraCapture* capture);
         bool GrabFrame(V4L2CameraCapture* capture);
         void V4L2Loop(V4L2CameraCapture* capture);
         int ReadFrame(V4L2CameraCapture* capture);
         IplImage* RetrieveFrame(V4L2CameraCapture* capture);
         bool MJPEG2RGB24(int width, int height, unsigned char *src, int length, unsigned char *dst);
         int SetControl(V4L2CameraCapture* capture);
         int GetControl(V4L2CameraCapture* capture);
   };
} /* namespace v4l2 */
#endif /* C920CAMERA_H_ */
