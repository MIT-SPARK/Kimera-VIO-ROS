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

	// Set frame count to 0 (Keeping track of number of frames processed)
	frame_count_ = 0;  

	// Get stereo matching parameters 
	stereo_matching_params_ = frontend_params_.getStereoMatchingParams();

	// Start IMU subscriber 
	imu_subscriber nh.subscribe(odom_topic, 10, &Navigator::poseCallback, this); 

	// Synchronize stero image callback 
  sync.registerCallback(boost::bind(&RosDataProvider::callbackCamAndProcessStereo, this, _1, _2) );
}

cv::Mat RosDataProvider::readRosImage(sensor_msgs::Image ros_img) {
	// Use cv_bridge to read ros image to cv::Mat
	cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptrLeft = cv_bridge::toCvCopy(ros_img);
  } catch(cv_bridge::Exception& exception) {
    ROS_ERROR("cv_bridge exception: %s", exception.what());
    return;
  }
  return cv_ptr->image; // Return cv::Mat
}

bool RosDataProvider::parseCameraData(StereoCalibration* stereo_calib) {
	// Parse camera calibration info (from param server)
	
}

bool RosDataProvider::parseImuData(ImuData* imudata) {
	// Parse IMU calibration info (from param server)

	/* NOTE (TODO) Might also need to parse the following: 
	imuData_.imu_rate_, imuData_.imu_rate_std_, imuData_.imu_rate_maxMismatch_
	imuData_.gyro_noise_, imuData_.gyro_walk_, imuData_.acc_noise_, imuData_.acc_walk_
	*/
}

// IMU callback 
void RosDataProvider::callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU){
	// Callback and store IMU data and timestamp until next StereoImuSyncPacket is made
  gtsam::Vector6 gyroAccData; 
  gtsam::Vector6 imu_accgyr;

  imu_accgyr(0) = msgIMU->linear_acceleration.x; 
  imu_accgyr(1) = msgIMU->linear_acceleration.y; 
  imu_accgyr(2) = msgIMU->linear_acceleration.z; 
  imu_accgyr(3) = msgIMU->angular_velocity.x; 
  imu_accgyr(4) = msgIMU->angular_velocity.y;
  imu_accgyr(5) = msgIMU->angular_velocity.z; 

  Timestamp timestamp = (msgIMU->header.stamp.sec * 1e9 + msgLeft->header.stamp.nsec);

  // add measurement to buffer (CHECK if this is OK, need to manually delete old data?)
  imudata->imu_buffer_.addMeasurement(timestamp, imu_accgyr);

}

// Callback for stereo images and main spin 
void callbackCamAndProcessStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                                 const sensor_msgs::ImageConstPtr& msgRight){
	// Main spin of the data provider: Interpolates IMU data and build StereoImuSyncPacket
	// (Think of this as the spin of the other parser/data-providers)
	cv::Mat left_image = readRosImage(msgLeft);
	cv::Mat right_image = readRosImage(msgRight);

	// Timestamp is in nanoseconds 
	Timestamp timestamp = (msgLeft->header.stamp.sec * 1e9 + msgLeft->header.stamp.nsec);

	ImuMeasurements imu_meas; 

  ROS_DEBUG_COND(utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable !=
        				 kitti_data_.imuData_.imu_buffer_.getImuDataInterpolatedUpperBorder(
        				 last_time_stamp_,
        				 timestamp,
        				 &imu_meas.timestamps_,
        				 &imu_meas.measurements_), "IMU data not available");

  // Call VIO Pipeline.
  ROS_INFO("Processing frame %d with timestamp: %d", frame_count_, timestamp)

  vio_callback_(StereoImuSyncPacket(
		                StereoFrame(k, timestamp_frame_k,
		                left_image,
		                stereo_calib_.left_camera_info_,
		                right_image,
		                stereo_calib_.right_camera_info_,
		                stereo_calib_.camL_pose_camR,
		                stereo_matching_params),
		                imu_meas.timestamps_,
		                imu_meas.measurements_));

  last_time_stamp_ = timestamp;
}

} // End of VIO namespace
