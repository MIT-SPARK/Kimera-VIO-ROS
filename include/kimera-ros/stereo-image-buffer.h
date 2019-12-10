/**
 * @file   stereo-image-buffer.h
 * @brief  Stereo Image Buffer for ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include <cstdio>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <kimera-vio/common/vio_types.h>
#include <kimera-vio/dataprovider/DataProviderInterface.h>

namespace VIO {

struct StereoPacket {
  Timestamp timestamp;
  sensor_msgs::ImageConstPtr left_ros_img;
  sensor_msgs::ImageConstPtr right_ros_img;
};

class StereoBuffer {
  std::vector<StereoPacket> stereo_buffer_;
  // Latest timestamp
  Timestamp latest_timestamp_;
  // Earliest timestamp
  Timestamp earliest_timestamp_;

 public:
  // Get timestamp of latest frame
  Timestamp getLatestTimestamp() const;
  Timestamp getEarliestTimestamp() const;
  size_t size() const;

  // Get the images of the latest frame
  // and also delete from buffer
  bool extractLatestImages(sensor_msgs::ImageConstPtr& left_img,
                           sensor_msgs::ImageConstPtr& right_img);

  // Add images to buffer
  // and update latest timestamp
  void addStereoFrame(const sensor_msgs::ImageConstPtr& left_img,
                      const sensor_msgs::ImageConstPtr& right_img);

  // Discard next frame
  void removeNext();
};

}  // namespace VIO
