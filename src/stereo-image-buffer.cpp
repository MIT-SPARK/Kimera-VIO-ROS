/**
 * @file   stereo-image-buffer.cpp
 * @brief  Stereo image buffer ROS wrapper
 * @author Yun Chang
 */
#include "spark-vio-ros/stereo-image-buffer.h"

VIO::Timestamp StereoBuffer::getEarliestTimestamp() const {
  return earliest_timestamp_;
}

VIO::Timestamp StereoBuffer::getLatestTimestamp() const {
  return latest_timestamp_;
}

size_t StereoBuffer::size() const {
  return stereo_buffer_.size();
}

bool StereoBuffer::extractLatestImages(sensor_msgs::ImageConstPtr& left_img,
                                       sensor_msgs::ImageConstPtr& right_img) {
  if (stereo_buffer_.size() == 0) {
    // no more images in buffer
    return true;
  }

  left_img = stereo_buffer_[0].left_ros_img;
  right_img = stereo_buffer_[0].right_ros_img;

  // discard this frame
  stereo_buffer_.erase(stereo_buffer_.begin());

  // update earliest time
  if (stereo_buffer_.size() > 0) {
    earliest_timestamp_ = stereo_buffer_[0].timestamp;
  }
  return true;
}

void StereoBuffer::addStereoFrame(sensor_msgs::ImageConstPtr left_img,
                                  sensor_msgs::ImageConstPtr right_img) {
  // Timestamp is in nanoseconds
  VIO::Timestamp timestamp = left_img->header.stamp.toNSec();

  if (stereo_buffer_.size() == 0) {
    // if only frame
    earliest_timestamp_ = timestamp;
  }

  // create packet
  StereoPacket sp;
  sp.timestamp = timestamp;
  sp.left_ros_img = left_img;
  sp.right_ros_img = right_img;

  // update buffer and time
  stereo_buffer_.push_back(sp);
  latest_timestamp_ = timestamp;
  return;
}

void StereoBuffer::removeNext() {
  // Remove next pair of stereo frames
  if (stereo_buffer_.size() > 1) {
    stereo_buffer_.erase(stereo_buffer_.begin());
    earliest_timestamp_ = stereo_buffer_[0].timestamp;
  }
}
