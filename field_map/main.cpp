#include <iostream>
#include <opencv2/opencv.hpp>

int main(void)
{
	cv::Mat image = cv::imread("zebracornsB&WField.PNG");
	std::cout << image.size() << " " << image.depth() << " " << image.channels() << std::endl;
	cv::Mat bw_image(cv::Size(image.cols/2+1, image.rows/2+1), CV_8UC1);

	int left_edge = image.cols;
	int right_edge = 0;
	for (int row = 0; row < image.rows; row+=2)
	{
		bool left_edge_seen = false;
		bool right_edge_seen = false;
		for (int col = 0; col < image.cols; col+=2)
		{
			if (image.at<cv::Vec3b>(row, col)[0] < 5)
			{
				bw_image.at<unsigned char>(row/2, col/2) = 0;
				if (left_edge_seen && !right_edge_seen)
				{
					right_edge = std::max(right_edge, col);
					right_edge_seen = true;
				}
			}
			else
			{
				bw_image.at<unsigned char>(row/2, col/2) = 255;
				if (!left_edge_seen)
				{
					left_edge = std::min(left_edge, col);
					left_edge_seen = true;
				}
			}
		}
		std::cerr << std::endl;
	}
	left_edge /= 2;
	right_edge /= 2;
	std::cout << left_edge << " "  << right_edge << std::endl;
	double width = right_edge - left_edge + 1;

	double meter_per_pixel = 16.46 / width;
	std::cout << "meters per pixel = " << meter_per_pixel << std::endl;

	std::cout << "width x height " << meter_per_pixel * bw_image.cols << " " << meter_per_pixel * bw_image.rows << std::endl;
	std::cout << "half width x height " << meter_per_pixel * bw_image.cols / 2 << " " << meter_per_pixel * bw_image.rows / 2 << std::endl;

	cv::imwrite("bwfield_half.png", bw_image);
	cv::imshow("Input", image);
	cv::imshow("Output", bw_image);
	cv::waitKey(0);
	return 0;
}

