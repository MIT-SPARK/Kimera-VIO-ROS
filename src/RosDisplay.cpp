/**
 * @file   RosDisplay.cpp
 * @brief  Publishes 2D data sent in the display_queue in Kimera. This publishes
 * images at any rate (frame rate, keyframe rate,...).
 * @author Antoni Rosinol
 */
#include "kimera_vio_ros/RosDisplay.h"

#include <string>

#include <glog/logging.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

#include <kimera-vio/visualizer/Display.h>

#include "kimera_vio_ros/RosPublishers.h"
#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosDisplay::RosDisplay()
    : DisplayBase(VIO::DisplayType::kOpenCV), nh_private_("~"), image_publishers_(nullptr) {
  image_publishers_ = std::make_unique<ImagePublishers>(nh_private_);
}

void RosDisplay::spinOnce(DisplayInputBase::UniquePtr&& viz_output) {
  CHECK(viz_output);
  // Display 2D images.
  spin2dWindow(*viz_output);
  // No need to run ros::spinOnce(), since it is done with async spinner.
}

void RosDisplay::spin2dWindow(const DisplayInputBase& viz_output) {
  for (const ImageToDisplay& img_to_display : viz_output.images_to_display_) {
    std_msgs::Header header;
    header.stamp.fromNSec(viz_output.timestamp_);
    header.frame_id = base_link_frame_id_;
    // Copies...
    image_publishers_->publish(
        img_to_display.name_,
        cv_bridge::CvImage(header, "bgr8", img_to_display.image_).toImageMsg());
  }
}

}  // namespace VIO
