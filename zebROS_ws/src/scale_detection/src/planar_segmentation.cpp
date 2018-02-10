#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

void callback (const ImageConstPtr &frameMsg, const ImageConstPtr &depthMsg){

  cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
  cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 10;
  cloud->height = 7;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the (random) data
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  // Show the point cloud data
  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // Set a limit on the distance of points
  seg.setDistanceThreshold (20);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  //Check if planar model can be extracted from given points
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  //Show the final model inliers (points that meet criteria)
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "goal_detect");
  ros::NodeHandle nh("~");
  int sub_rate = 5;
  message_filters::Subscriber<Image> frame_sub(nh, "/zed_goal/left/image_rect_color", sub_rate);
  message_filters::Subscriber<Image> depth_sub(nh, "/zed_goal/depth/depth_registered", sub_rate);
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy2;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(xxx)
  Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(50), frame_sub, depth_sub);
  sync2.registerCallback(boost::bind(&callback, _1, _2));

}
