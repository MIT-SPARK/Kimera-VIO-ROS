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

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "RosBaseDataSource.h"

namespace VIO {

class RosbagDataProvider: public RosBaseDataProvider {
public:
  RosbagDataProvider(std::string left_camera_topic,
                     std::string right_camera_topic,
                     std::string imu_topic,
                     std::string bag_input_path);
  virtual ~RosbagDataProvider();
  bool spin();

private:
  // Parse rosbag data
  bool parseRosbag(std::string bag_path,
                   std::string left_imgs_topic,
                   std::string right_imgs_topic,
                   std::string imu_topic,
                   RosbagData* data);

  // Print the parameters
  void print() const;

private:
  RosbagData rosbag_data_;
};

} // End of VIO Namespace
