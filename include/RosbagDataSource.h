/**
 * @file   RosDataSource.h
 * @brief  ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include <string>
#include <functional>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include "datasource/DataSource.h"
#include "StereoImuSyncPacket.h"
#include "StereoFrame.h"
#include "VioFrontEndParams.h"
#include "ImuFrontEnd.h"
#include "ETH_parser.h"
#include "pipeline/Pipeline.h"

// ROS Dependencies
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include "nav_msgs/Odometry.h"
#include <rosbag/bag.h>

namespace VIO {

class RosbagDataProvider: public DataProvider {
public:
  RosbagDataProvider(std::string left_camera_topic, 
                  std::string right_camera_topic, 
                  std::string imu_topic);
  virtual ~RosbagDataProvider();
  virtual bool spin();

private:  
	ImuData imuData_; // store IMU data from last frame 
  ImuParams imuParams_;
	Timestamp last_time_stamp_; // Timestamp correponding to last frame
  int frame_count_; // Keep track of number of frames processed  

	// Stereo info 
  struct StereoCalibration{
  	CameraParams left_camera_info_;
  	CameraParams right_camera_info_;
  	gtsam::Pose3 camL_Pose_camR_; // relative pose between cameras
  };

private:
  // Parse camera calibration info (from param server)
  bool parseCameraData(StereoCalibration* stereo_calib);

  // Parse IMU calibration info
  bool parseImuData(ImuData* imudata, ImuParams* imuparams);

  void publishOutput(gtsam::Pose3 pose, gtsam::Vector3 velocity, Timestamp ts) const;
  
  // Print the parameters 
  void print() const;

private:
  VioFrontEndParams frontend_params_; 
  StereoCalibration stereo_calib_; 
};

} // End of VIO Namespace 