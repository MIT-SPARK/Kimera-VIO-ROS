/**
 * @file   RosDisplay.h
 * @brief  Publishes 2D data sent in the display_queue in Kimera. This publishes
 * images at any rate (frame rate, keyframe rate,...).
 * @author Antoni Rosinol
 */
#pragma once

#include <string>

#include <ros/ros.h>

#include <kimera-vio/visualizer/Display.h>

#include "kimera_vio_ros/RosPublishers.h"

namespace VIO {

class RosDisplay : public DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RosDisplay);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDisplay);

 public:
  RosDisplay();
  virtual ~RosDisplay() = default;

 public:
  /**
   * @brief spinOnce
   * Spins the display once to render the visualizer output.
   * @param viz_output
   */
  void spinOnce(DisplayInputBase::UniquePtr&& viz_output) override;

 protected:
  /**
   * @brief spin2dWindow Publishes images to ROS
   * @param viz_output Input from the display queue
   */
  void spin2dWindow(const DisplayInputBase& viz_output);

 private:
  //! Ros
  ros::NodeHandle nh_private_;

  //! Define image publishers manager
  std::unique_ptr<ImagePublishers> image_publishers_;

  //! Define frame ids for odometry message
  std::string base_link_frame_id_;
};

}  // namespace VIO
