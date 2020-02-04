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
  frame_count_(VIO::FrameId(0)),
  vio_pipeline_(this->pipeline_params_),
  last_imu_timestamp_(0),
  last_stereo_timestamp_(0)
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
  auto qos = rclcpp::SensorDataQoS();
  imu_sub_ = this->create_subscription<Imu>(
    imu_topic,
    qos,
    std::bind(
      &KimeraVioNode::imu_cb,
      this,
      std::placeholders::_1),
    imu_opt);
  
  std::string transport = "raw";
  std::string left_topic = "left_cam";
  std::string right_topic = "right_cam";
  image_transport::TransportHints hints(this, transport);
  int queue_size_ = 10;

//  left_sub_.subscribe(this, left_topic, hints.getTransport(), qos.get_rmw_qos_profile());
//  right_sub_.subscribe(this, right_topic, hints.getTransport(), qos.get_rmw_qos_profile());
//  exact_sync_ = std::make_shared<ExactSync>(
//      ExactPolicy(queue_size_), left_sub_, right_sub_);

  l_sub_ = std::make_shared<message_filters::Subscriber<Image> >(
            std::shared_ptr<rclcpp::Node>(this),
            left_topic, qos.get_rmw_qos_profile());
  r_sub_ = std::make_shared<message_filters::Subscriber<Image> >(
            std::shared_ptr<rclcpp::Node>(this),
            right_topic, qos.get_rmw_qos_profile());
  exact_sync_ = std::make_shared<ExactSync>(
      ExactPolicy(queue_size_), *l_sub_, *r_sub_);

  exact_sync_->registerCallback(&KimeraVioNode::stereo_cb, this);

    // Register callback for inputs.
    this->registerImuSingleCallback(
            std::bind(&VIO::Pipeline::fillSingleImuQueue,
                      &vio_pipeline_,
                      std::placeholders::_1));

    this->registerImuMultiCallback(
            std::bind(&VIO::Pipeline::fillMultiImuQueue,
                      &vio_pipeline_,
                      std::placeholders::_1));

    this->registerLeftFrameCallback(
            std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                      &vio_pipeline_,
                      std::placeholders::_1));

    this->registerRightFrameCallback(
            std::bind(&VIO::Pipeline::fillRightFrameQueue,
                      &vio_pipeline_,
                      std::placeholders::_1));

    vio_pipeline_.registerBackendOutputCallback(
            std::bind(&VIO::RosDataProviderInterface::callbackBackendOutput,
                    this,
                      std::placeholders::_1));

    vio_pipeline_.registerFrontendOutputCallback(
            std::bind(&VIO::RosDataProviderInterface::callbackFrontendOutput,
                      this,
                      std::placeholders::_1));

    vio_pipeline_.registerMesherOutputCallback(
            std::bind(&VIO::RosDataProviderInterface::callbackMesherOutput,
                      this,
                      std::placeholders::_1));

    bool use_lcd;
    this->declare_parameter("use_lcd", false);
    this->get_parameter("use_lcd", use_lcd);
    if (use_lcd) {
        vio_pipeline_.registerLcdOutputCallback(
                std::bind(&VIO::RosDataProviderInterface::callbackLcdOutput,
                          this,
                          std::placeholders::_1));
    }
    handle_pipeline_ = std::async(std::launch::async,
                                      &VIO::Pipeline::spin,
                                      &vio_pipeline_);
}

KimeraVioNode::~KimeraVioNode()
{
    vio_pipeline_.shutdown();
    handle_pipeline_.get();
}

void KimeraVioNode::stereo_cb(
    const Image::SharedPtr left_msg,
    const Image::SharedPtr right_msg)
{
  rclcpp::Time left_stamp(left_msg->header.stamp);
  rclcpp::Time right_stamp(right_msg->header.stamp);
  if(left_stamp.nanoseconds() > last_stereo_timestamp_.nanoseconds())
  {
  static const VIO::CameraParams& left_cam_info =
      pipeline_params_.camera_params_.at(0);
  static const VIO::CameraParams& right_cam_info =
      pipeline_params_.camera_params_.at(1);

  const VIO::Timestamp& timestamp_left = left_stamp.nanoseconds();
  const VIO::Timestamp& timestamp_right = right_stamp.nanoseconds();

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
  LOG_EVERY_N(INFO, 30) << "Done: KimeraVioNode::stereo_cb";
  frame_count_++;
}
  last_stereo_timestamp_ = left_stamp;
}

void KimeraVioNode::imu_cb(const Imu::SharedPtr imu_msg)
{
  rclcpp::Time stamp(imu_msg->header.stamp);
  if(stamp.nanoseconds() > last_imu_timestamp_.nanoseconds())
  {
  VIO::ImuAccGyr imu_accgyr;

  imu_accgyr(0) = imu_msg->linear_acceleration.x;
  imu_accgyr(1) = imu_msg->linear_acceleration.y;
  imu_accgyr(2) = imu_msg->linear_acceleration.z;
  imu_accgyr(3) = imu_msg->angular_velocity.x;
  imu_accgyr(4) = imu_msg->angular_velocity.y;
  imu_accgyr(5) = imu_msg->angular_velocity.z;

  // Adapt imu timestamp to account for time shift in IMU-cam
  VIO::Timestamp timestamp = stamp.nanoseconds();

//  static const ros::Duration imu_shift(pipeline_params_.imu_params_.imu_shift_);
//  if (imu_shift != ros::Duration(0)) {
//    LOG_EVERY_N(WARNING, 1000) << "imu_shift is not 0.";
//    timestamp -= imu_shift.toNSec();
//  }

  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
//  TODO: Use RCLCPP_INFO inplace of CHECK?
  // RCLCPP_INFO(this->get_logger(), "Did you forget to register the IMU callback?");

  imu_single_callback_(VIO::ImuMeasurement(timestamp, imu_accgyr));
  LOG_EVERY_N(INFO, 200) << "Done: KimeraVioNode::imu_cb";
}
  last_imu_timestamp_ = stamp;
}
