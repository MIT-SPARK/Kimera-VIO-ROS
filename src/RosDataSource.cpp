/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RosDataSource.cpp
 * @brief  ROS wrapper
 * @author Yun Chang
 */
#include "RosDataSource.h"

namespace VIO {

RosDataProvider::RosDataProvider(): 
			it_(nh_cam_),
			left_img_subscriber_(it_, left_camera_topic_, 1), // Image subscriber (left)
			right_img_subscriber_(it_, right_camera_topic_, 1), // Image subscriber (right)
			sync(sync_pol(10), left_img_subscriber_, right_img_subscriber_) {

	// Parse topic names from parameter server 
	nh_.getParam("left_camera_topic", left_camera_topic_);
	nh_.getParam("right_camera_topic", right_camera_topic_); 
	nh_.getParam("imu_topic", imu_topic_); 

	// Parse calibration info for camera and IMU 
	// Calibration info on parameter server (Parsed from yaml)
	parseCameraData(&stereo_calib_);
	parseImuData(&imuData_);

	// Start IMU callback 
	// TODO!!!

	// Synchronize stero image callback 
  sync.registerCallback(boost::bind(&RosDataProvider::callbackCamAndProcessStereo, this, _1, _2) );
}

cv::Mat RosDataProvider::readRosImage(const std::string& img_name) {
	// Use cv_bridge to read ros image to openCV 

}

bool RosDataProvider::parseCameraData(StereoCalibration* stereo_calib) {
	// Parse camera calibration info (from param server)

}

bool RosDataProvider::parseImuData(ImuData* imudata) {
	// Parse IMU calibration info (from param server)

}

// IMU callback 
void RosDataProvider::callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU){
	// Callback and store IMU data and timestamp until next StereoImuSyncPacket is made

}

// Callback for stereo images and main spin 
void callbackCamAndProcessStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                                 const sensor_msgs::ImageConstPtr& msgRight){
	// Main spin of the data provider: Interpolates IMU data and build StereoImuSyncPacket

}

} // End of VIO namespace
