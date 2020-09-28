/**
 * @file   RosBagDataProvider.h
 * @brief  Parse rosbag and run Kimera-VIO.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <functional>
#include <string>

#include <opencv2/opencv.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/logging/Logger.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"

namespace VIO {

struct RosbagData {
  /// IMU messages
  std::vector<sensor_msgs::ImuConstPtr> imu_msgs_;
  /// The names of the images from left camera
  std::vector<sensor_msgs::ImageConstPtr> left_imgs_;
  /// The names of the images from right camera
  std::vector<sensor_msgs::ImageConstPtr> right_imgs_;
  /// Vector of timestamps see issue in .cpp file
  std::vector<Timestamp> timestamps_;
  /// Ground-truth Odometry (only if available)
  std::vector<nav_msgs::OdometryConstPtr> gt_odometry_;
};

class RosbagDataProvider : public RosDataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosbagDataProvider);
  KIMERA_POINTER_TYPEDEFS(RosbagDataProvider);

  RosbagDataProvider(const VioParams& vio_params);
  virtual ~RosbagDataProvider() = default;

  /**
   * @brief initialize The Rosbag data provider: it will parse the Rosbag and
   * call the VIO IMU callback (so you need to register the imu cb before), and
   * it will parse the gt odom for initialization if requested.
   */
  void initialize();

  // Returns true if the whole rosbag was successfully played, false if ROS was
  // shutdown before the rosbag finished.
  bool spin() override;

 private:
  // Parse rosbag data
  // Optionally, send a ground-truth odometry topic if available in the rosbag.
  // If gt_odom_topic is empty (""), it will be ignored.
  bool parseRosbag(const std::string& bag_path, RosbagData* rosbag_data);

  void sendImuDataToVio();

  // Get ground-truth nav state for VIO initialization.
  // It uses odometry messages inside of the rosbag as ground-truth (indexed
  // by the sequential order in the rosbag).
  VioNavState getGroundTruthVioNavState(const size_t& k_frame) const;

  void publishRosbagInfo(const Timestamp& timestamp);

  // Publish clock
  void publishClock(const Timestamp& timestamp) const;

  // Publish raw input data to ROS at keyframe rate
  void publishInputs(const Timestamp& timestamp_kf);

  // Publish outputs
  void publishOutputs();

  // Log output gt csv
  void logGtData(const nav_msgs::OdometryConstPtr& odometry);

 private:
  RosbagData rosbag_data_;

  std::string rosbag_path_;
  std::string left_imgs_topic_;
  std::string right_imgs_topic_;
  std::string imu_topic_;
  std::string gt_odom_topic_;

  ros::Publisher clock_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher left_img_pub_;
  ros::Publisher right_img_pub_;
  ros::Publisher gt_odometry_pub_;

  Timestamp timestamp_last_frame_;
  Timestamp timestamp_last_kf_;
  Timestamp timestamp_last_imu_;
  Timestamp timestamp_last_gt_;

  // Frame indices
  //! Left frame index
  size_t k_;
  size_t k_last_kf_;
  size_t k_last_imu_;
  size_t k_last_gt_;

  bool log_gt_data_;
  bool is_header_written_poses_vio_;
  OfstreamWrapper output_gt_poses_csv_;
};

}  // namespace VIO
