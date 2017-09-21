// From : https://raw.githubusercontent.com/Smorodov/Multitarget-tracker/master/KalmanFilter/Kalman.h
#pragma once
#include <opencv2/opencv.hpp>
//#include <Eigen/Geometry>
// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
class TKalmanFilter
{
	public:
		TKalmanFilter(const cv::Point3f &p, float dt = 0.05, float Accel_noise_mag = 0.5);
		cv::Point3f GetPrediction();
		cv::Point3f Update(const cv::Point3f &p);
	//	void adjustPrediction(const Eigen::Transform<double, 3, Eigen::Isometry> &delta_robot);
		void adjustPrediction(const cv::Point3f &delta_pos);
	private:
		cv::KalmanFilter kalman;
};

