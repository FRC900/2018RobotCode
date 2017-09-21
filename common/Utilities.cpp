#include "Utilities.hpp"
using namespace std;

namespace utils {

	std::pair<float, float> minOfDepthMat(const cv::Mat& img, const cv::Mat& mask, const cv::Rect& bound_rect, int range) {

		if (img.empty())
			return make_pair(-1,-1);
		if ((img.rows != mask.rows) || (img.cols != mask.cols))
			return make_pair(-2,-2);

		//cout << bound_rect << endl;
		float min = numeric_limits<float>::max();
		float max = numeric_limits<float>::min();
		int min_loc_x;
		int min_loc_y;
		int max_loc_x;
		int max_loc_y;
		bool found = false;
		for (int j = bound_rect.tl().y; j <= bound_rect.br().y; j++) //for each row
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);

			for (int i = bound_rect.tl().x; i <= bound_rect.br().x; i++) //for each pixel in row
			{
				//cout << i << " " << j << " " << ptr_img[i] << " " << (int)ptr_mask[i] << endl;
				if (ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
				{
					found = true;
					if (ptr_img[i] > max)
					{
						max = ptr_img[i];
						max_loc_x = i;
						max_loc_y = j;
					}

					if (ptr_img[i] < min)
					{
						min = ptr_img[i];
						min_loc_x = i;
						min_loc_y = j;
					}
				}
			}
		}
		if(!found)
		{
			return make_pair(-3, -3);
		}
		float sum_min   = 0;
		int num_pix_min = 0;
		for (int j = min_loc_y - range; j < (min_loc_y + range); j++)
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);
		    for (int i = min_loc_x - range; i < (min_loc_x + range); i++)
		    {
		        if ((0 < i) && (i < img.cols) && (0 < j) && (j < img.rows) && ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
		        {
		            sum_min += ptr_img[i];
		            num_pix_min++;
		        }
		    }
		}
		float sum_max = 0;
		int num_pix_max = 0;
		for (int j = max_loc_y - range; j < (max_loc_y + range); j++)
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);
		    for (int i = max_loc_x - range; i < (max_loc_x + range); i++)
		    {
		        if ((0 < i) && (i < img.cols) && (0 < j) && (j < img.rows) && ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
		        {
		            sum_max += ptr_img[i];
		            num_pix_max++;
		        }
		    }
		}
		// Need to debug this more but for now fix it 
		// by returning a negative number (i.e. failure)
		// if it happens
		float min_dist = sum_min / num_pix_min;
		if (isinf(min_dist))
			min_dist = -4;
		float max_dist = sum_max / num_pix_max;
		if (isinf(max_dist))
			max_dist = -4;
		return std::make_pair(min_dist, max_dist);
	}

	float avgOfDepthMat(const cv::Mat& img, const cv::Mat& mask, const cv::Rect& bound_rect) 
	{
		double sum = 0.0;
		unsigned count = 0;
		for (int j = bound_rect.tl().y; j <= bound_rect.br().y; j++) //for each row
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);

			for (int i = bound_rect.tl().x; i <= bound_rect.br().x; i++) //for each pixel in row
			{
				if (ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
				{
					sum += ptr_img[i];
					count += 1;
				}
			}
		}
		if (count == 0)
			return -1;
		return sum / count;
	}

	void shrinkRect(cv::Rect &rect_in, float shrink_factor) {

		rect_in.tl() = rect_in.tl() + cv::Point(shrink_factor/2.0 * rect_in.width, shrink_factor/2.0 * rect_in.height);
		rect_in.br() = rect_in.br() - cv::Point(shrink_factor/2.0 * rect_in.width, shrink_factor/2.0 * rect_in.height);

	}
#if 0
	void printIsometry(const Eigen::Transform<double, 3, Eigen::Isometry> m) {

		Eigen::Vector3d xyz = m.translation();
		Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
		cout << "Camera Translation: " << xyz << endl;
		cout << "Camera Rotation: " << rpy << endl;
		
	}
#endif

	//gets the slope that the masked area is facing away from the camera
	//useful when used with a contour to find the angle that an object is faciing

	double slope_list(const std::vector<double>& x, const std::vector<double>& y) {
	    const auto n    = x.size();
	    const auto s_x  = std::accumulate(x.begin(), x.end(), 0.0);
	    const auto s_y  = std::accumulate(y.begin(), y.end(), 0.0);
	    const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
	    const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
	    const auto a    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
	    return a;
	}

	std::pair<double,double> slopeOfMasked(ObjectType ot, const cv::Mat &depth, const cv::Mat &mask, cv::Point2f fov) {

		CV_Assert(mask.depth() == CV_8U);
		vector<double> slope_x_values;
		vector<double> slope_y_values;
		vector<double> slope_z_values;

		for (int j = 0; j < depth.rows; j++) {

			const float *ptr_depth = depth.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);

			for (int i = 0; i < depth.cols; i++) {
				if (ptr_mask[i] && (ptr_depth[i] > 0)) {
					cv::Point3f pos = ot.screenToWorldCoords(cv::Rect(i,j,0,0), ptr_depth[i], fov, depth.size(), 0);
					slope_x_values.push_back(pos.x);
					slope_y_values.push_back(pos.y);
					slope_z_values.push_back(pos.z);
				}
			}
		}

		return std::make_pair<double,double>(slope_list(slope_x_values, slope_y_values), slope_list(slope_z_values, slope_y_values));

	}
	
	double normalCFD(const pair<double,double> &meanAndStddev, double value)
	{
		double z_score = (value - meanAndStddev.first) / meanAndStddev.second;
   		return 0.5 * erfc(-z_score * M_SQRT1_2);
	}
	
	class DataRecorder {
		public:	
			DataRecorder(void) {}
			
			DataRecorder(const string &file_name, const vector<string> &column_names) {
				_data_file.open(file_name + ".csv");
				_num_columns = column_names.size();
				log(column_names);
			}

			~DataRecorder() { _data_file.close(); }

			void log(const vector<string> &data) {
				//this function won't do anything if the data file was not opened
				//this makes it safe to not pass a DataRecorder to an object and it won't break everything

				if(_data_file.is_open()) {
					if(data.size() != _num_columns)
						cerr << "Bad info log!" << endl;
					for(size_t i = 0; i < data.size(); i++)
						_data_file << data[i] << ",";
					_data_file << "\n";
				}
			}
		private:
			size_t _num_columns;
			ofstream _data_file;
	};

}
