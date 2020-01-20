#include "kimera_vio_ros/KimeraVioNode.hpp"

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

std::string KimeraVioNodeNode::timing_string()
{
  rclcpp::Time time = this->now();
  return std::to_string(time.nanoseconds());
}

KimeraVioNodeNode::KimeraVioNodeNode()
: Node("KimeraVioNodeNode")
{
  callback_group_subscriber1_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_subscriber2_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  auto sub1_opt = rclcpp::SubscriptionOptions();
  sub1_opt.callback_group = callback_group_subscriber1_;
  auto sub2_opt = rclcpp::SubscriptionOptions();
  sub2_opt.callback_group = callback_group_subscriber2_;

  subscription1_ = this->create_subscription<std_msgs::msg::String>(
    "topic",
    rclcpp::QoS(10),
    std::bind(
      &KimeraVioNodeNode::subscriber1_cb,
      this,
      std::placeholders::_1),
    sub1_opt);

  subscription2_ = this->create_subscription<std_msgs::msg::String>(
    "topic",
    rclcpp::QoS(10),
    std::bind(
      &KimeraVioNodeNode::subscriber2_cb,
      this,
      std::placeholders::_1),
    sub2_opt);
}

void KimeraVioNodeNode::subscriber1_cb(const std_msgs::msg::String::SharedPtr msg)
{
  auto message_received_at = timing_string();

  // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
  thread_string += " => Heard '%s' at " + message_received_at;
  RCLCPP_INFO(this->get_logger(), thread_string, msg->data.c_str());
}

void KimeraVioNodeNode::subscriber2_cb(const std_msgs::msg::String::SharedPtr msg)
{
  auto message_received_at = timing_string();

  // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
  thread_string += " => Heard '%s' at " + message_received_at;
  RCLCPP_INFO(this->get_logger(), thread_string, msg->data.c_str());
}
