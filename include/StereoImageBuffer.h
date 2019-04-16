/**
 * @file   StereoImageBuffer.h
 * @brief  Stereo Image Buffer for ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include <cstdio>
#include <vector>

// ROS Dependencies
#include <ros/ros.h>
#include <ros/console.h>
#include "sensor_msgs/Image.h"

struct StereoPacket{
  long int timestamp;
  sensor_msgs::ImageConstPtr left_ros_img;
  sensor_msgs::ImageConstPtr right_ros_img;
};

class StereoBuffer {

  std::vector<StereoPacket> stereo_buffer_;
  // Latest timestamp
  long int latest_timestamp_;
  // Earliest timestamp
  long int earliest_timestamp_;

public:
  // Get timestamp of latest frame
  long int get_latest_timestamp() const;
  long int get_earliest_timestamp() const;
  size_t size() const;

  // Get the images of the latest frame
  // and also delete from buffer
  bool extract_latest_images(sensor_msgs::ImageConstPtr& left_img,
                             sensor_msgs::ImageConstPtr& right_img);

  // add images to buffer
  // and update latest timestamp
  void add_stereo_frame(sensor_msgs::ImageConstPtr left_img,
                        sensor_msgs::ImageConstPtr right_img);

  void remove_next();
  // discard next frame
};
