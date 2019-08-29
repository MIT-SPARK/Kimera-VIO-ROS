/**
 * @file   rosbag-data-source.h
 * @brief  ROS wrapper
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include "spark-vio-ros/base-data-source.h"

namespace VIO {

struct RosbagData {
  inline size_t getNumberOfImages() const { return left_imgs_.size(); }
  // The names of the images from left camera
  std::vector<sensor_msgs::ImageConstPtr> left_imgs_;
  // The names of the images from right camera
  std::vector<sensor_msgs::ImageConstPtr> right_imgs_;
  // Vector of timestamps see issue in .cpp file
  std::vector<Timestamp> timestamps_;
  // IMU data
  ImuData imu_data_;
  // Ground-truth Odometry (only if available).
  std::vector<nav_msgs::OdometryConstPtr> gt_odometry_;
};

class RosbagDataProvider : public RosBaseDataProvider {
 public:
  RosbagDataProvider();
  virtual ~RosbagDataProvider();

  // Returns true if the whole rosbag was successfully played, false if ROS was
  // shutdown before the rosbag finished.
  virtual bool spin() override;

 private:
  // Parse rosbag data
  // Optionally, send a ground-truth odometry topic if available in the rosbag.
  // If gt_odom_topic is empty (""), it will be ignored.
  bool parseRosbag(const std::string& bag_path,
                   const std::string& left_imgs_topic,
                   const std::string& right_imgs_topic,
                   const std::string& imu_topic,
                   const std::string& gt_odom_topic,
                   RosbagData* data);

  // Parse IMU calibration info (for rosbag)
  bool parseImuData(RosbagData* rosbag_data, ImuParams* imuparams);

  // Get ground-truth nav state for VIO initialization.
  // It uses odometry messages inside of the rosbag as ground-truth (indexed
  // by the sequential order in the rosbag).
  VioNavState getGroundTruthVioNavState(const size_t& k_frame) const;

  // Publish clock
  void publishClock(const Timestamp& timestamp) const;

  // Publish ground-truth odometry
  void publishGroundTruthOdometry(
      const nav_msgs::OdometryConstPtr& gt_odom) const;

  // Print the parameters
  void print() const;

 private:
  RosbagData rosbag_data_;
  ros::Publisher clock_pub_;
  ros::Publisher gt_odometry_pub_;
};

}  // namespace VIO
