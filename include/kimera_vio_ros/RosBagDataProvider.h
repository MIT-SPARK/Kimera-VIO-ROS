/**
 * @file   rosbag-data-source.h
 * @brief  ROS wrapper
 * @author Yun Chang
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <functional>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"

namespace VIO {

struct RosbagData {
  // IMU messages
  std::vector<sensor_msgs::ImuConstPtr> imu_msgs_;
  // The names of the images from left camera
  std::vector<sensor_msgs::ImageConstPtr> left_imgs_;
  // The names of the images from right camera
  std::vector<sensor_msgs::ImageConstPtr> right_imgs_;
  // Vector of timestamps see issue in .cpp file
  std::vector<Timestamp> timestamps_;
  // Ground-truth Odometry (only if available)
  std::vector<nav_msgs::OdometryConstPtr> gt_odometry_;
};

class RosbagDataProvider : public RosDataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosbagDataProvider);
  KIMERA_POINTER_TYPEDEFS(RosbagDataProvider);

  RosbagDataProvider();
  virtual ~RosbagDataProvider() = default;

  // Returns true if the whole rosbag was successfully played, false if ROS was
  // shutdown before the rosbag finished.
  bool spin() override;

  // bool spinOnce();

 private:
  // Parse rosbag data
  // Optionally, send a ground-truth odometry topic if available in the rosbag.
  // If gt_odom_topic is empty (""), it will be ignored.
  bool parseRosbag(const std::string& bag_path, RosbagData* rosbag_data);

  // Get ground-truth nav state for VIO initialization.
  // It uses odometry messages inside of the rosbag as ground-truth (indexed
  // by the sequential order in the rosbag).
  VioNavState getGroundTruthVioNavState(const size_t& k_frame) const;

  void publishBackendOutput(const BackendOutput::Ptr& output) override;

  // Publish clock
  void publishClock(const Timestamp& timestamp) const;

  // Publish raw input data to ROS at keyframe rate
  void publishInputs(const Timestamp& timestamp_kf);

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

  Timestamp timestamp_last_kf_;
  Timestamp timestamp_last_imu_;
  Timestamp timestamp_last_gt_;
  size_t k_last_kf_;
  size_t k_last_imu_;
  size_t k_last_gt_;
};

}  // namespace VIO
