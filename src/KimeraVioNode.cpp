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

KimeraVioNode::KimeraVioNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
: VIO::RosDataProviderInterface(node_name, options),
  frame_count_(VIO::FrameId(0))
{
  callback_group_stereo_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_imu_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  auto stereo_opt = rclcpp::SubscriptionOptions();
  stereo_opt.callback_group = callback_group_stereo_;
  auto imu_opt = rclcpp::SubscriptionOptions();
  imu_opt.callback_group = callback_group_imu_;

  std::string imu_topic = "imu";
  imu_sub_ = this->create_subscription<Imu>(
    imu_topic,
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
  const Image::ConstSharedPtr& left_msg,
  const Image::ConstSharedPtr& right_msg)
{
  static const VIO::CameraParams& left_cam_info =
      pipeline_params_.camera_params_.at(0);
  static const VIO::CameraParams& right_cam_info =
      pipeline_params_.camera_params_.at(1);

  const VIO::Timestamp& timestamp_left = left_msg->header.stamp.nanosec;
  const VIO::Timestamp& timestamp_right = right_msg->header.stamp.nanosec;

  CHECK(left_frame_callback_)
  << "Did you forget to register the left frame callback?";
  CHECK(right_frame_callback_)
  << "Did you forget to register the right frame callback?";
//  TODO: Use RCLCPP_INFO inplace of CHECK?
  // RCLCPP_INFO(this->get_logger(), "Did you forget to register the right frame callback?");

  left_frame_callback_(VIO::make_unique<VIO::Frame>(
      frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));
  right_frame_callback_(VIO::make_unique<VIO::Frame>(
      frame_count_, timestamp_right, right_cam_info, readRosImage(right_msg)));

  frame_count_++;
}

void KimeraVioNode::imu_cb(const Imu::SharedPtr imu_msg)
{
  VIO::ImuAccGyr imu_accgyr;

  imu_accgyr(0) = imu_msg->linear_acceleration.x;
  imu_accgyr(1) = imu_msg->linear_acceleration.y;
  imu_accgyr(2) = imu_msg->linear_acceleration.z;
  imu_accgyr(3) = imu_msg->angular_velocity.x;
  imu_accgyr(4) = imu_msg->angular_velocity.y;
  imu_accgyr(5) = imu_msg->angular_velocity.z;

  // Adapt imu timestamp to account for time shift in IMU-cam
  VIO::Timestamp timestamp = imu_msg->header.stamp.nanosec;

//  static const ros::Duration imu_shift(pipeline_params_.imu_params_.imu_shift_);
//  if (imu_shift != ros::Duration(0)) {
//    LOG_EVERY_N(WARNING, 1000) << "imu_shift is not 0.";
//    timestamp -= imu_shift.toNSec();
//  }

  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
//  TODO: Use RCLCPP_INFO inplace of CHECK?
  // RCLCPP_INFO(this->get_logger(), "Did you forget to register the IMU callback?");

  imu_single_callback_(VIO::ImuMeasurement(timestamp, imu_accgyr));
}
