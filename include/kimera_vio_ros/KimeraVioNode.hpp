// #include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class KimeraVioNode : public rclcpp::Node{
public:
    KimeraVioNode();

private:
    void subscriber1_cb(const std_msgs::msg::String::SharedPtr msg);
    void subscriber2_cb(const std_msgs::msg::String::SharedPtr msg);
    std::string timing_string();

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
};
