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
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace VIO {

class RosbagDataProvider: public DataProvider {
public:
  RosbagDataProvider(std::string left_camera_topic,
                     std::string right_camera_topic,
                     std::string imu_topic,
                     std::string bag_input_path);
  virtual ~RosbagDataProvider();
  virtual bool spin();

  inline ImuParams getImuParams() const {
    return imuParams_;
  }

private:
  // Define Node Handler for Parameter server
  ros::NodeHandle nh_;

  ImuParams imuParams_;
  Timestamp last_time_stamp_; // Timestamp correponding to last frame
  int frame_count_; // Keep track of number of frames processed

  // Stereo info
  struct StereoCalibration{
    CameraParams left_camera_info_;
    CameraParams right_camera_info_;
    gtsam::Pose3 camL_Pose_camR_; // relative pose between cameras
  };

  struct Data {
    inline size_t getNumberOfImages() const {return left_imgs_.size();}
    // The image names of the images from left camera
    std::vector<sensor_msgs::ImageConstPtr> left_imgs_;
    // The image names of the images from right camera
    std::vector<sensor_msgs::ImageConstPtr> right_imgs_;
    // Vector of timestamps see issue in .cpp file
    std::vector<Timestamp> timestamps_;
    //IMU data
    ImuData imuData_;
  };

private:
  cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg);

  // Parse camera calibration info (from param server)
  bool parseCameraData(StereoCalibration* stereo_calib);

  // Parse IMU calibration info
  bool parseImuData(Data* data, ImuParams* imuparams);

  // Parse rosbag data
  bool parseRosbag(std::string bag_path,
                   std::string left_imgs_topic,
                   std::string right_imgs_topic,
                   std::string imu_topic,
                   Data* data);

  // Print the parameters
  void print() const;

private:
  VioFrontEndParams frontend_params_;
  StereoCalibration stereo_calib_;
  Data data_;
};

} // End of VIO Namespace
