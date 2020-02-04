// #include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

// Still need gflags for parameters in VIO
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "image_transport/subscriber_filter.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <kimera-vio/pipeline/Pipeline.h>
#include "kimera_vio_ros/RosDataProviderInterface.hpp"


using namespace message_filters::sync_policies;
using namespace sensor_msgs::msg;

 class KimeraVioNode : public VIO::RosDataProviderInterface{
public:
    KimeraVioNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~KimeraVioNode();

private:
   VIO::FrameId frame_count_;

  void stereo_cb(
      const Image::SharedPtr left_msg,
      const Image::SharedPtr right_msg);
  void imu_cb(const Imu::SharedPtr imu_msg);
  std::string timing_string();

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_stereo_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_imu_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;

  typedef ExactTime<Image, Image> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  std::shared_ptr<ExactSync> exact_sync_;
  image_transport::SubscriberFilter left_sub_, right_sub_;

  std::shared_ptr<message_filters::Subscriber<Image> > l_sub_;
  std::shared_ptr<message_filters::Subscriber<Image> > r_sub_;

  VIO::Pipeline vio_pipeline_;
  std::future<bool> handle_pipeline_;

  rclcpp::Time last_imu_timestamp_;
  rclcpp::Time last_stereo_timestamp_;
};
