/**
 * @file   StereoImageBuffer.cpp
 * @brief  Stereo image buffer ROS wrapper
 * @author Yun Chang
 */
#include "StereoImageBuffer.h"

long int StereoBuffer::get_earliest_timestamp() const {
  return earliest_timestamp_;
}

long int StereoBuffer::get_latest_timestamp() const {
  return latest_timestamp_;
}

size_t StereoBuffer::size() const {
  return stereo_buffer_.size();
}

bool StereoBuffer::extract_latest_images(sensor_msgs::ImageConstPtr& left_img,
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

void StereoBuffer::add_stereo_frame(sensor_msgs::ImageConstPtr left_img,
                                    sensor_msgs::ImageConstPtr right_img) {
  // Timestamp is in nanoseconds
  long double sec = (long double) left_img->header.stamp.sec;
  long double nsec = (long double) left_img->header.stamp.nsec;
  long int timestamp = (long int) (sec * 1e9 + nsec);

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

void StereoBuffer::remove_next() {
  // Remove next pair of stereo frames
  if (stereo_buffer_.size() > 1) {
    stereo_buffer_.erase(stereo_buffer_.begin());
    earliest_timestamp_ = stereo_buffer_[0].timestamp;
  }
}
