/**
 * @file   stereo-image-buffer.h
 * @brief  Stereo Image Buffer for ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include <cstdio>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <common/vio_types.h>

#include "datasource/DataSource.h"

struct StereoPacket {
  VIO::Timestamp timestamp;
  sensor_msgs::ImageConstPtr left_ros_img;
  sensor_msgs::ImageConstPtr right_ros_img;
};

class StereoBuffer {
  std::vector<StereoPacket> stereo_buffer_;
  // Latest timestamp
  VIO::Timestamp latest_timestamp_;
  // Earliest timestamp
  VIO::Timestamp earliest_timestamp_;

public:
  // Get timestamp of latest frame
  VIO::Timestamp getLatestTimestamp() const;
  VIO::Timestamp getEarliestTimestamp() const;
  size_t size() const;

  // Get the images of the latest frame
  // and also delete from buffer
  bool extractLatestImages(sensor_msgs::ImageConstPtr& left_img,
                           sensor_msgs::ImageConstPtr& right_img);

  // add images to buffer
  // and update latest timestamp
  void addStereoFrame(sensor_msgs::ImageConstPtr left_img,
                      sensor_msgs::ImageConstPtr right_img);

  void removeNext();
  // discard next frame
};
