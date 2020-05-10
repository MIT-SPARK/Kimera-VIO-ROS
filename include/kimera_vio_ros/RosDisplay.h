#pragma once

#include <string>
#include <unordered_map>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <kimera-vio/visualizer/Display.h>

namespace VIO {

class RosDisplay : public DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RosDisplay);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDisplay);

 public:
  RosDisplay() :
    DisplayBase(),
    nh_private_("~"),
    image_publishers_(nullptr) {
    image_publishers_ = VIO::make_unique<ImagePublishers>(nh_private_);
  }
  virtual ~RosDisplay() = default;

 public:
  // Spins the display once to render the visualizer output.
  virtual void spinOnce(VisualizerOutput::UniquePtr&& viz_output) {
    CHECK(viz_output);
    // Display 2D images.
    spin2dWindow(*viz_output);
    // ros::spinOnce();
  }

 protected:
  void spin2dWindow(const VisualizerOutput& viz_output) {
    for (const ImageToDisplay& img : viz_output.images_to_display_) {
      std_msgs::Header header;
      header.frame_id = "base_link_frame_id_";  // TODO TONI REMOVE!
      // header.stamp.fromNSec(ros::Time::now());
      header.stamp = ros::Time::now(); // TODO TONI TIMESTAMP!
      image_publishers_->publish(
          img.name_,
          cv_bridge::CvImage(header, "bgr8", img.image_).toImageMsg());
    }
  }

 private:
  ros::NodeHandle nh_private_;
  // Define image publishers manager
  std::unique_ptr<ImagePublishers> image_publishers_;
};

}  // namespace VIO
