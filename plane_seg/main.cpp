#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "zedsvoin.hpp"
#include "zedcamerain.hpp"
#include "ZvSettings.hpp"

using namespace cv;
using namespace std;

int main(void)
{
	//MediaIn *cap = new ZedSVOIn("/home/kjaget/Downloads/wall.svo");
	ZvSettings zvSettings("settings.xml");
	MediaIn *cap = new ZedCameraIn(true, &zvSettings);

	Mat frame;
	Mat depth;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	Mat mask;

	namedWindow("parameters", 1);
	Scalar min;
	Scalar max;

	int hLo = 29;
	int sLo = 57;
	int vLo = 120;
	int hUp = 38;
	int sUp = 187;
	int vUp = 255;

	createTrackbar("HLo", "parameters", &hLo, 179);
	createTrackbar("SLo", "parameters", &sLo, 255);
	createTrackbar("VLo", "parameters", &vLo, 255);
	createTrackbar("HUp", "parameters", &hUp, 179);
	createTrackbar("SUp", "parameters", &sUp, 255);
	createTrackbar("VUp", "parameters", &vUp, 255);


	pcl::ModelCoefficients coefficients;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (150);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	while (cap->getFrame(frame, depth, cloud))
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudP(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
		const size_t origSize = cloudP->points.size();



		while (cloudP->points.size() > (0.1 * origSize))
		{
			seg.setInputCloud (cloudP);
			seg.segment (*inliers, coefficients);
			if (inliers->indices.size () == 0)
			{
				cerr << "Could not estimate a planar model for the given dataset." << endl;
				break;
			}
			cerr << "Model coefficients: " 
				<< coefficients.values[0] << " " 
				<< coefficients.values[1] << " "
				<< coefficients.values[2] << " " 
				<< coefficients.values[3] << endl;

			cerr << "Model inliers: " << inliers->indices.size () << endl;
			
			Mat rgb(0, 1, CV_8UC3);

			for (size_t i = 0; i < inliers->indices.size (); ++i) {
				rgb.push_back(Vec3b(cloud.points[inliers->indices[i]].b, cloud.points[inliers->indices[i]].g, cloud.points[inliers->indices[i]].r));
			}
			Mat hsv;
			
			cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
			cv::inRange(hsv, min, max, mask);
			
			int nonZero = countNonZero(mask);
			cerr << "HSV Mat Size " << nonZero << endl;

			min = Scalar(hLo, sLo, vLo);
			max = Scalar(hUp, sUp, vUp);
			cerr << hLo << endl;

			// Filter out inliner points 
			// to get a set of points not in the
			// detected plane. Use those the next
			// time through the loop to try and find
			// another plane
			extract.setInputCloud (cloudP);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*out);
			cloudP.swap(out);
#if 0
			for (size_t i = 0; i < inliers.indices.size (); ++i)
				cerr << inliers.indices[i] << "    " 
					<< cloud.points[inliers.indices[i]].x << " "
					<< cloud.points[inliers.indices[i]].y << " "
					<< cloud.points[inliers.indices[i]].z << endl;
#endif
		}
		Mat reg;
		Mat mask2;
		cv::cvtColor(frame, reg, cv::COLOR_BGR2HSV);
		cv::inRange(reg, min, max, mask2);
		imshow("Window", frame);
		imshow("mask2", mask2);
		if ((uchar)waitKey(5) == 27)
			break;
	}
	return 0;
}
