#!/usr/bin/perl
#
# Script used to install RoboRIO ipk packages into a cross-root development
# environment.  
#
use strict;
use File::Temp;
use Cwd;

sub install_package
{
	my $url = @_[0];

	my $oldcwd = getcwd;
	# Create a temporary directory
	my $dirname = File::Temp->newdir;
    # Go to the temporary directory
	chdir $dirname  || die("Can not chdir($dirname) : $!");
	`wget $url`;

	# ipk files are an ar archive. Inside is a file
	# called data.tar.gz which is the actual contents
	# that have to be extracted. Put them in the
	# /usr/arm-frc-linux-gnueabi sysroot directory
	if (rindex($url, "/") != -1)
	{
		my $filename = substr $url, rindex($url, "/")+1;
		`ar xv $filename`;
		print `sudo tar xzvf data.tar.gz -C /usr/arm-frc-linux-gnueabi`;
	}
	
	chdir $oldcwd;
}

# These are all needed to build ROS from source for the RoboRIO.  They
# might not all be needed to build programs against the ROS code ...
# some are static libs which get linked into ROS programs.  Still,
# be safe rather than sorry since disk space isn't at a huge premium 
# on our development systems.
#


install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/lz4-dev_131+git0+d86dc91677-r0.4_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/lz4-staticdev_131+git0+d86dc91677-r0.4_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/lz4_131+git0+d86dc91677-r0.4_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libcurl4_7.51.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/curl-dev_7.51.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/curl-staticdev_7.51.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/curl_7.51.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/boost-dev_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/boost-serialization_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/boost-staticdev_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/boost-test_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/boost_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-atomic1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-chrono1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-date-time1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-filesystem1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-graph1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-iostreams1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-program-options1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-regex1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-signals1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-system1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libboost-thread1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libeigen-dev_3.2.6-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libeigen_3.2.6-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libpython2_2.7.11-r1.49_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/python-core_2.7.11-r1.49_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/python-dev_2.7.11-r1.49_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libcurl4_7.51.0-r0.3_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/bzip2-dev_1.0.6-r5.326_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/bzip2-staticdev_1.0.6-r5.326_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/bzip2_1.0.6-r5.326_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libbz2-1_1.0.6-r5.326_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/cmake-dev_3.4.3-r0.3_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/cmake_3.4.3-r0.3_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libxml2-dev_2.9.4-r0.49_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libxml2-staticdev_2.9.4-r0.49_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libxml2_2.9.4-r0.49_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgnutls-bin_3.4.9-r0.7_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgnutls-dev_3.4.9-r0.7_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgnutls-openssl27_3.4.9-r0.7_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgnutls30_3.4.9-r0.7_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgnutlsxx28_3.4.9-r0.7_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/nettle-dev_3.2-r0.7_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/nettle-staticdev_3.2-r0.7_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/nettle_3.2-r0.7_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgmp-dev_6.1.0-r0.44_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgmp-staticdev_6.1.0-r0.44_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgmp10_6.1.0-r0.44_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libgmpxx4_6.1.0-r0.44_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libz-dev_1.2.8-r0.329_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libz-staticdev_1.2.8-r0.329_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libz1_1.2.8-r0.329_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/protobuf-dev_2.6.1-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/protobuf-staticdev_2.6.1-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/protobuf_2.6.1-r0.5_cortexa9-vfpv3.ipk");

# OpenCV - needed for WPILib
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-calib3d-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-calib3d3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-core-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-core3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-features2d-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-features2d3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-flann-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-flann3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-highgui-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-highgui3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-imgcodecs-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-imgcodecs3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-imgproc-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-imgproc3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-ml-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-ml3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-objdetect-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-objdetect3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-photo-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-photo3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-shape-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-shape3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-stitching-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-stitching3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-superres-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-superres3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-video-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-video3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-videoio-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-videoio3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-videostab-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libopencv-videostab3.1_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/opencv-apps_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/opencv-dbg_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/opencv-dev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/opencv-samples-dbg_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/opencv-samples_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/opencv-staticdev_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/opencv_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/python-opencv_3.1+git0+92387b1ef8-r0.5_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/tbb-dev_4.1-r20130314.8_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/tbb_4.1-r20130314.8_cortexa9-vfpv3.ipk");

# rsync packages needed for deployment script
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libacl1_2.2.52-r0.105_cortexa9-vfpv3.ipk");
install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/rsync_3.1.2-r0.3_cortexa9-vfpv3.ipk");

install_package("http://download.ni.com/ni-linux-rt/feeds/2017/arm/ipk/cortexa9-vfpv3/libidn11_1.32-r0.7_cortexa9-vfpv3.ipk");
