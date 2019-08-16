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
  bool parseRosbag(const std::string& bag_path,
                   const std::string& left_imgs_topic,
                   const std::string& right_imgs_topic,
                   const std::string& imu_topic, RosbagData* data);

  // Parse IMU calibration info (for rosbag)
  bool parseImuData(RosbagData* rosbag_data, ImuParams* imuparams);

  // Print the parameters
  void print() const;

 private:
  RosbagData rosbag_data_;
};

}  // namespace VIO
