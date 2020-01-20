// #include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "image_transport/subscriber_filter.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"


using namespace message_filters::sync_policies;
using namespace sensor_msgs::msg;

class KimeraVioNode : public rclcpp::Node{
public:
    KimeraVioNode();

private:

    void stereo_cb(
        const Image::ConstSharedPtr& left,
        const Image::ConstSharedPtr& right);
    void imu_cb(const Imu::SharedPtr msg);
    std::string timing_string();

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_stereo_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_imu_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    
    typedef ExactTime<Image, Image> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    std::shared_ptr<ExactSync> exact_sync_;
    image_transport::SubscriberFilter left_sub_, right_sub_;

};
