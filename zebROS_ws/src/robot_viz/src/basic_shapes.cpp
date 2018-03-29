#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	image_transport::ImageTransport it(n);
	image_transport::Publisher background_pub = it.advertise("background_image", 1);

    uint32_t shape = visualization_msgs::Marker::CUBE;

    // Read in field image
	cv::Mat background_image = cv::imread("../img/background.jpeg", CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr background = cv_bridge::CvImage(std_msgs::Header(), "bgr8", background_image).toImageMsg();

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        sensor_msgs::Image background;

        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes"; // Any marker sent with the same namespace and id will overwrite the old one
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

		// Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
        background_pub.publish(background);

        // Cycle between different shapes
        switch (shape)
        {
        case visualization_msgs::Marker::CUBE:
            shape = visualization_msgs::Marker::SPHERE;
            break;
        case visualization_msgs::Marker::SPHERE:
            shape = visualization_msgs::Marker::ARROW;
            break;
        case visualization_msgs::Marker::ARROW:
            shape = visualization_msgs::Marker::CYLINDER;
            break;
        case visualization_msgs::Marker::CYLINDER:
            shape = visualization_msgs::Marker::CUBE;
            break;
        }

        r.sleep();
    }
}
