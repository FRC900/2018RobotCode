#pragma once
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION == 2
#include <opencv2/gpu/gpu.hpp>
using cv::gpu::GpuMat;
#elif CV_MAJOR_VERSION == 3
#include <opencv2/core/cuda.hpp>
using cv::cuda::GpuMat;
#endif


