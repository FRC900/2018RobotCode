#include <iostream>

#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>

#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "navXTimeSync/AHRS.h"
#include "navx_publisher/stampedUInt64.h"
#include <tf/transform_datatypes.h>

using namespace std;
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{
	{
		0.1, 0, 0, 0, 0, 0,
		0, 0.1, 0, 0, 0, 0,
		0, 0, 0.1, 0, 0, 0,
		0, 0, 0, 0.17, 0, 0,
		0, 0, 0, 0, 0.17, 0,
		0, 0, 0, 0, 0, 0.17
	}
};
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{
	{
		0.05, 0, 0, 0, 0, 0,
		0, 0.05, 0, 0, 0, 0,
		0, 0, 0.05, 0, 0, 0,
		0, 0, 0, 0.09, 0, 0,
		0, 0, 0, 0, 0.09, 0,
		0, 0, 0, 0, 0, 0.09
	}
};

static const boost::array<double, 9> STANDARD_ORIENTATION_COVARIANCE =
{
	{
		0.00015, 0,       0,
		0,       0.00015, 0,
		0,       0,       0.00015
	}
};

static const boost::array<double, 9> STANDARD_VELOCITY_COVARIANCE =
{
	{
		0.0015, 0,     0,
		0,     0.0015, 0,
		0,     0,      0.0015
	}
};

static const boost::array<double, 9> STANDARD_ACCELERATION_COVARIANCE =
{
	{
		0.05, 0,    0,
		0,    0.05, 0,
		0,    0,    0.05
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navx_publisher");

	ros::NodeHandle nh;
	// Set up publishers
	// Raw_pub publishes in the ENU (east north up) orientation
	// instead of NED (north east down)
	ros::Publisher time_pub = nh.advertise<navx_publisher::stampedUInt64>("/navx/time", 75);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/navx/imu", 75);
	ros::Publisher raw_pub = nh.advertise<sensor_msgs::Imu>("/navx/raw", 75);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/navx/odom", 75);
	navx_publisher::stampedUInt64 timestamp;
	sensor_msgs::Imu imu_msg;
	sensor_msgs::Imu imu_msg_raw;
	nav_msgs::Odometry odom;

	imu_msg.linear_acceleration_covariance = imu_msg_raw.linear_acceleration_covariance = STANDARD_ACCELERATION_COVARIANCE;
	imu_msg.angular_velocity_covariance = imu_msg_raw.angular_velocity_covariance = STANDARD_VELOCITY_COVARIANCE;
	imu_msg.orientation_covariance = imu_msg_raw.orientation_covariance = STANDARD_ORIENTATION_COVARIANCE;

	odom.twist.covariance = STANDARD_TWIST_COVARIANCE;
	odom.pose.covariance = STANDARD_POSE_COVARIANCE;

	imu_msg_raw.header.frame_id = "navx_frame";
	imu_msg.header.frame_id = "navx_frame";
	odom.header.frame_id = "nav_current_frame";

	{
		//read the file with covariances and apply it to the odometry and IMU
		ifstream infile("/home/ubuntu/2017VisionCode/zebROS_ws/src/navx_publisher/navx_calib.dat");
		if (!infile.good())
			cerr << "navx_calib.dat file not opened!" << endl;
		std::string line;
		int ln = 0;
		while (std::getline(infile, line))
		{
			if (line == "") break;
			imu_msg.linear_acceleration_covariance[ln] = std::stod(line);
			ln++;
		}
		ln = 0;
		while (std::getline(infile, line))
		{
			if (line == "") break;
			imu_msg.angular_velocity_covariance[ln] = std::stod(line);
			ln++;
		}
		ln = 0;
		while (std::getline(infile, line))
		{
			if (line == "") break;
			imu_msg.orientation_covariance[ln] = std::stod(line);
			ln++;
		}
		ln = 0;
		while (std::getline(infile, line))
		{
			if (line == "") break;
			odom.twist.covariance[ln] = std::stod(line);
			odom.pose.covariance[ln] = std::stod(line);
			ln++;
		}
	}
	imu_msg_raw.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance;
	imu_msg_raw.angular_velocity_covariance = imu_msg.angular_velocity_covariance;
	imu_msg_raw.orientation_covariance = imu_msg.orientation_covariance;

	ros::Time last_time;
	tf::Quaternion last_rot (tf::Vector3(0., 0., 0.), 0.);

	bool firstrun = true;

	ros::Rate loop_time(210);
	AHRS nx("/dev/ttyACM0", AHRS::SerialDataType::kProcessedData, 200);
	nx.ZeroYaw();
	while (ros::ok())
	{
		unsigned long long nxstamp = nx.GetLastSensorTimestamp();
		if (firstrun || (nxstamp != timestamp.data))
		{
			//set the timestamp for all headers
			odom.header.stamp =
				imu_msg.header.stamp =
					imu_msg_raw.header.stamp =
						timestamp.header.stamp = ros::Time::now();

			float nx_roll;
			float nx_pitch;
			float nx_yaw;
			float nx_qx;
			float nx_qy;
			float nx_qz;
			float nx_qw;
			float nx_ax;
			float nx_ay;
			float nx_az;
			long  nx_stamp;

			//pull orientation data from NavX
			//all in one shot
			nx.GetRPYQAccel(nx_roll, nx_pitch, nx_yaw,
							nx_qx, nx_qy, nx_qz, nx_qw,
							nx_ax, nx_ay, nx_az,
							nx_stamp);

			nx_roll  *=  -M_PI / 180.;
			nx_pitch *=   M_PI / 180.;
			nx_yaw   *=  -M_PI / 180.;

			// roll and pitch are swapped from navx convention to the ros expected one
			tf::Quaternion q = tf::createQuaternionFromRPY(nx_pitch, nx_roll, nx_yaw);

			//std::cout << "From NX : " << nx_qx << " " << nx_qy << " " << nx_qz << " " << nx_qw << std::endl;
			//std::cout << "From RPY : " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;


			tf::quaternionTFToMsg(q, imu_msg.orientation);
			imu_msg.linear_acceleration.x = nx_ax;
			imu_msg.linear_acceleration.y = nx_ay;
			imu_msg.linear_acceleration.z = nx_az;
			timestamp.data = nx_stamp;

			imu_msg_raw.orientation.x = imu_msg.orientation.x;
			imu_msg_raw.orientation.y = imu_msg.orientation.y;
			imu_msg_raw.orientation.z = -imu_msg.orientation.z;
			imu_msg_raw.orientation.w = imu_msg.orientation.w;

			const double grav = 9.80665;
			imu_msg.linear_acceleration.x *= grav;
			imu_msg.linear_acceleration.y *= grav;
			imu_msg.linear_acceleration.z *= grav;

#if 1
			imu_msg_raw.linear_acceleration = imu_msg.linear_acceleration;
#else
			//uncomment this to add gravity back into /navx/raw
			nx_yaw   *= M_PI / 180.;
			nx_pitch *= M_PI / 180.;
			nx_roll  *= M_PI / 180.;
			imu_msg_raw.linear_acceleration.x = imu_msg.linear_acceleration.x + sin(nx_roll) * cos(nx_pitch) * grav;
			imu_msg_raw.linear_acceleration.y = imu_msg.linear_acceleration.y + cos(nx_roll) * sin(nx_pitch) * grav;
			imu_msg_raw.linear_acceleration.z = imu_msg.linear_acceleration.z + cos(nx_pitch) * cos(nx_roll) * grav;
#endif

			tf::Quaternion pose;
			double yaw;
			double pitch;
			double roll;
			tf::quaternionMsgToTF(imu_msg_raw.orientation, pose); // or imu_msg? they differ in the z value
			if (firstrun) last_rot = pose;
			tf::Quaternion rot = pose * last_rot.inverse();
			tf::Matrix3x3(rot).getRPY(roll, pitch, yaw);
			const double dTime = odom.header.stamp.toSec() - last_time.toSec();
			imu_msg.angular_velocity.x = roll / dTime;
			imu_msg.angular_velocity.y = pitch / dTime;
			imu_msg.angular_velocity.z = -yaw / dTime;
			imu_msg_raw.angular_velocity = imu_msg.angular_velocity;
			last_rot = pose;
			last_time = odom.header.stamp;

			firstrun = false;

			//pull position data (all this is the integral of velocity so it's not very good)
			odom.pose.pose.position.x = nx.GetDisplacementX();
			odom.pose.pose.position.y = nx.GetDisplacementY();
			odom.pose.pose.position.z = nx.GetDisplacementZ();
			odom.pose.pose.orientation = imu_msg_raw.orientation; // or imu_msg? see above question

			odom.twist.twist.linear.x = nx.GetVelocityX();
			odom.twist.twist.linear.y = nx.GetVelocityY();
			odom.twist.twist.linear.z = nx.GetVelocityZ();

			odom.twist.twist.angular = imu_msg.angular_velocity;

			//publish to ROS topics
			time_pub.publish(timestamp);
			imu_pub.publish(imu_msg);
			odom_pub.publish(odom);
			raw_pub.publish(imu_msg_raw);
		}
		ros::spinOnce();
		loop_time.sleep();
	}

	return 0;
}
