/**
 * @file   rosbag-data-source.h
 * @brief  ROS wrapper
 * @author Yun Chang
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

#include "spark-vio-ros/ros-base-data-source.h"

namespace VIO {

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

  // Print the parameters
  void print() const;

 private:
  RosbagData rosbag_data_;
};

}  // namespace VIO
