#include "kimera_vio_ros/KimeraVioNode.hpp"

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

std::string KimeraVioNode::timing_string()
{
  rclcpp::Time time = this->now();
  return std::to_string(time.nanoseconds());
}

KimeraVioNode::KimeraVioNode()
: Node("KimeraVioNode")
{
  callback_group_stereo_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_imu_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  auto stereo_opt = rclcpp::SubscriptionOptions();
  stereo_opt.callback_group = callback_group_stereo_;
  auto imu_opt = rclcpp::SubscriptionOptions();
  imu_opt.callback_group = callback_group_imu_;

  imu_sub_ = this->create_subscription<Imu>(
    "topic",
    rclcpp::QoS(10),
    std::bind(
      &KimeraVioNode::imu_cb,
      this,
      std::placeholders::_1),
    imu_opt);
  
  std::string transport = "raw";
  std::string left_topic = "left_cam";
  std::string right_topic = "right_cam";
  image_transport::TransportHints hints(this, transport);

  left_sub_.subscribe(this, left_topic, hints.getTransport());
  right_sub_.subscribe(this, right_topic, hints.getTransport());

  int queue_size_ = 10;
  exact_sync_.reset( new ExactSync(
    ExactPolicy(queue_size_),
    left_sub_,
    right_sub_) );
   exact_sync_->registerCallback(
     std::bind(
       &KimeraVioNode::stereo_cb,
       this,
       std::placeholders::_1,
       std::placeholders::_2));
}

void KimeraVioNode::stereo_cb(
  const Image::ConstSharedPtr& left,
  const Image::ConstSharedPtr& right)
{
  auto message_received_at = timing_string();

  // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
  thread_string += " => Heard '%s' at " + message_received_at;
  // RCLCPP_INFO(this->get_logger(), thread_string, msg->data.c_str());
}

void KimeraVioNode::imu_cb(const Imu::SharedPtr msg)
{
  auto message_received_at = timing_string();

  // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
  thread_string += " => Heard '%s' at " + message_received_at;
  // RCLCPP_INFO(this->get_logger(), thread_string, msg->data.c_str());
}
