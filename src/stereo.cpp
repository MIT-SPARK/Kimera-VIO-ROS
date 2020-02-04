#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "image_transport/subscriber_filter.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
//#include "message_filters/sync_policies/approximate_time.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace message_filters::sync_policies;
using namespace sensor_msgs::msg;

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
            : Node("minimal_subscriber")
    {

        std::string transport = "raw";
        std::string left_topic = "/camera/infra1/image_rect_raw";
        std::string right_topic = "/camera/infra2/image_rect_raw";
        image_transport::TransportHints hints(this, transport);

        auto qos = rclcpp::SensorDataQoS();
        left_sub_.subscribe(this, left_topic, hints.getTransport(), qos.get_rmw_qos_profile());
        right_sub_.subscribe(this, right_topic, hints.getTransport(), qos.get_rmw_qos_profile());
//
//        int queue_size_ = 10;
//        exact_sync_.reset( new ExactSync(
//                ExactPolicy(queue_size_),
//                left_sub_,
//                right_sub_) );
//        exact_sync_->registerCallback(
//                std::bind(
//                        &MinimalSubscriber::topic_callback,
//                        this,
//                        std::placeholders::_1,
//                        std::placeholders::_2));


//        l_sub_ = std::make_shared<message_filters::Subscriber<Image> >(
//                std::shared_ptr<rclcpp::Node>(this),
//                left_topic, qos.get_rmw_qos_profile());
//        r_sub_ = std::make_shared<message_filters::Subscriber<Image> >(
//                std::shared_ptr<rclcpp::Node>(this),
//                right_topic, qos.get_rmw_qos_profile());

        int queue_size_ = 10;
//        exact_sync_ = std::make_shared<ExactSync>(
//                ExactPolicy(queue_size_), *l_sub_, *r_sub_);
        exact_sync_ = std::make_shared<ExactSync>(
                ExactPolicy(queue_size_), left_sub_, right_sub_);
        exact_sync_->registerCallback(&MinimalSubscriber::topic_callback, this);
    }

private:
    void topic_callback(
            const Image::SharedPtr left_msg,
            const Image::SharedPtr right_msg)
    {

        rclcpp::Time left_stamp(left_msg->header.stamp);
        rclcpp::Time right_stamp(right_msg->header.stamp);

        RCLCPP_INFO(this->get_logger(), "l_stamp: '%d'", left_stamp);
        RCLCPP_INFO(this->get_logger(), "r_stamp: '%r'", right_stamp);
    }
    typedef ExactTime<Image, Image> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    std::shared_ptr<ExactSync> exact_sync_;
    image_transport::SubscriberFilter left_sub_, right_sub_;

    std::shared_ptr<message_filters::Subscriber<Image> > l_sub_;
    std::shared_ptr<message_filters::Subscriber<Image> > r_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}