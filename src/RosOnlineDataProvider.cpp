/**
 * @file   RosOnlineDataProvider.cpp
 * @brief  ROS wrapper for online processing.
 * @author Antoni Rosinol
 */

#include "kimera_vio_ros/RosOnlineDataProvider.h"

#include <string>
#include <vector>

#include <glog/logging.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

namespace VIO {

RosOnlineDataProvider::RosOnlineDataProvider(const VioParams& vio_params)
    : RosDataProviderInterface(vio_params),
      frame_count_(FrameId(0)),
      left_img_subscriber_(),
      right_img_subscriber_(),
      imu_subscriber_(),
      gt_odom_subscriber_(),
      reinit_flag_subscriber_(),
      reinit_pose_subscriber_(),
      imu_queue_(),
      imu_async_spinner_(nullptr),
      async_spinner_(nullptr) {
  // Define ground truth odometry Subsrciber
  static constexpr size_t kMaxGtOdomQueueSize = 1u;
  if (vio_params_.backend_params_->autoInitialize_ == 0) {
    LOG(INFO) << "Requested initialization from ground-truth. "
              << "Initializing ground-truth odometry one-shot subscriber.";
    gt_odom_subscriber_ =
        nh_.subscribe("gt_odom",
                      kMaxGtOdomQueueSize,
                      &RosOnlineDataProvider::callbackGtOdomOnce,
                      this);

    LOG(WARNING) << "Waiting for ground-truth pose to initialize VIO "
                 << "on ros topic: " << gt_odom_subscriber_.getTopic().c_str();
    // We wait for the gt pose for kMaxIterations, if ros is running (aka
    // if time is non-zero (as would happen if running with sim_time).
    static constexpr size_t kMaxIterations = 100u;
    size_t iterations = 0u;
    while (!gt_init_pose_received_ &&
           ros::Time::now().isValid() && // Wait until time is non-zero
           iterations < kMaxIterations) {
      if (nh_.ok() && ros::ok() && !ros::isShuttingDown() && !shutdown_) {
        ros::spinOnce();
      } else {
        LOG(FATAL) << "Ros is not ok... Shutting down.";
      }
      ++iterations;
    }

    LOG_IF(WARNING, !gt_init_pose_received_)
        << "Missing ground-truth pose while trying for "
        << kMaxIterations << " times."
        << "Enabling autoInitialize and continuing without ground-truth "
           "pose.";
    vio_params_.backend_params_->autoInitialize_ = true;
  }

  static constexpr size_t kMaxImuQueueSize = 1000u;
  // Create a dedicated queue for the Imu callback so that we can use an async
  // spinner on it to process the data lighting fast.
  ros::SubscribeOptions imu_subscriber_options =
      ros::SubscribeOptions::create<sensor_msgs::Imu>(
          "imu",
          kMaxImuQueueSize,
          boost::bind(&RosOnlineDataProvider::callbackIMU, this, _1),
          ros::VoidPtr(),
          &imu_queue_);
  imu_subscriber_options.transport_hints.tcpNoDelay(true);

  // Start IMU subscriber
  imu_subscriber_ = nh_.subscribe(imu_subscriber_options);

  // Imu Async Spinner: will process the imu queue only, instead of ROS' global
  // queue. A value of 0 means to use the number of processor cores.
  static constexpr size_t kImuSpinnerThreads = 2u;
  imu_async_spinner_ =
      VIO::make_unique<ros::AsyncSpinner>(kImuSpinnerThreads, &imu_queue_);

  // Subscribe to stereo images. Approx time sync, should be exact though...
  // We set the queue to only 1, since we prefer to drop messages to reach
  // real-time than to be delayed...
  static constexpr size_t kMaxImagesQueueSize = 1u;
  CHECK(it_);
  left_img_subscriber_.subscribe(*it_, "left_cam", kMaxImagesQueueSize);
  right_img_subscriber_.subscribe(*it_, "right_cam", kMaxImagesQueueSize);
  static constexpr size_t kMaxImageSynchronizerQueueSize = 1u;
  sync_ = VIO::make_unique<message_filters::Synchronizer<sync_pol>>(
      sync_pol(kMaxImageSynchronizerQueueSize),
      left_img_subscriber_,
      right_img_subscriber_);
  DCHECK(sync_);
  sync_->registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackStereoImages, this, _1, _2));

  // Define Reinitializer Subscriber
  static constexpr size_t kMaxReinitQueueSize = 1u;
  reinit_flag_subscriber_ =
      nh_.subscribe("reinit_flag",
                    kMaxReinitQueueSize,
                    &RosOnlineDataProvider::callbackReinit,
                    this);
  reinit_pose_subscriber_ =
      nh_.subscribe("reinit_pose",
                    kMaxReinitQueueSize,
                    &RosOnlineDataProvider::callbackReinitPose,
                    this);

  // This async spinner will process the regular Global callback queue of ROS.
  static constexpr size_t kGlobalSpinnerThreads = 2u;
  async_spinner_ = VIO::make_unique<ros::AsyncSpinner>(kGlobalSpinnerThreads);
}

RosOnlineDataProvider::~RosOnlineDataProvider() {
  VLOG(1) << "RosDataProvider destructor called.";
}

// TODO(marcus): with the readRosImage, this is a slow callback. Might be too
// slow...
void RosOnlineDataProvider::callbackStereoImages(
    const sensor_msgs::ImageConstPtr& left_msg,
    const sensor_msgs::ImageConstPtr& right_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 2u);
  const CameraParams& left_cam_info =
      vio_params_.camera_params_.at(0);
  const CameraParams& right_cam_info =
      vio_params_.camera_params_.at(1);

  CHECK(left_msg);
  CHECK(right_msg);
  const Timestamp& timestamp_left = left_msg->header.stamp.toNSec();
  const Timestamp& timestamp_right = right_msg->header.stamp.toNSec();

  CHECK(left_frame_callback_)
      << "Did you forget to register the left frame callback?";
  CHECK(right_frame_callback_)
      << "Did you forget to register the right frame callback?";

  if (!shutdown_) {
    left_frame_callback_(VIO::make_unique<Frame>(
        frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));
    right_frame_callback_(VIO::make_unique<Frame>(
        frame_count_, timestamp_right, right_cam_info, readRosImage(right_msg)));
    frame_count_++;
  }
}

void RosOnlineDataProvider::callbackIMU(
    const sensor_msgs::ImuConstPtr& msgIMU) {

  // TODO(TONI): detect jump backwards in time?

  VIO::ImuAccGyr imu_accgyr;

  imu_accgyr(0) = msgIMU->linear_acceleration.x;
  imu_accgyr(1) = msgIMU->linear_acceleration.y;
  imu_accgyr(2) = msgIMU->linear_acceleration.z;
  imu_accgyr(3) = msgIMU->angular_velocity.x;
  imu_accgyr(4) = msgIMU->angular_velocity.y;
  imu_accgyr(5) = msgIMU->angular_velocity.z;

  // Adapt imu timestamp to account for time shift in IMU-cam
  Timestamp timestamp = msgIMU->header.stamp.toNSec();

  const ros::Duration imu_shift(vio_params_.imu_params_.imu_shift_);
  if (imu_shift != ros::Duration(0)) {
    LOG_EVERY_N(WARNING, 1000) << "imu_shift is not 0.";
    timestamp -= imu_shift.toNSec();
  }

  if (!shutdown_) {
    CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
  }
}

// Ground-truth odometry callback
void RosOnlineDataProvider::callbackGtOdomOnce(
    const nav_msgs::Odometry::ConstPtr& msgGtOdom) {
  LOG(WARNING) << "Using initial ground-truth state for initialization.";
  msgGtOdomToVioNavState(
      msgGtOdom,
      &vio_params_.backend_params_->initial_ground_truth_state_);

  // Signal receptance of ground-truth pose.
  gt_init_pose_received_ = true;

  // Shutdown subscriber to prevent new gt poses from interfering
  gt_odom_subscriber_.shutdown();
}

// Reinitialization callback
void RosOnlineDataProvider::callbackReinit(
    const std_msgs::Bool::ConstPtr& reinitFlag) {
  // TODO(Sandro): Do we want to reinitialize at specific pose or just at
  // origin? void RosDataProvider::callbackReinit( const
  // nav_msgs::Odometry::ConstPtr& msgReinit) {

  // Set reinitialization to "true"
  reinit_flag_ = true;

  if (getReinitFlag()) {
    ROS_INFO("Reinitialization flag received!\n");
  }
}

// Getting re-initialization pose
void RosOnlineDataProvider::callbackReinitPose(
    const geometry_msgs::PoseStamped& reinitPose) {
  // Set reinitialization pose
  gtsam::Rot3 rotation(gtsam::Quaternion(reinitPose.pose.orientation.w,
                                         reinitPose.pose.orientation.x,
                                         reinitPose.pose.orientation.y,
                                         reinitPose.pose.orientation.z));
  gtsam::Point3 position(reinitPose.pose.position.x,
                         reinitPose.pose.position.y,
                         reinitPose.pose.position.z);
  reinit_packet_.setReinitPose(gtsam::Pose3(rotation, position));
}

void RosOnlineDataProvider::msgGtOdomToVioNavState(
    const nav_msgs::Odometry::ConstPtr& gt_odom,
    VioNavState* vio_navstate) {
  CHECK_NOTNULL(vio_navstate);

  // World to Body rotation
  gtsam::Rot3 W_R_B = gtsam::Rot3::Quaternion(gt_odom->pose.pose.orientation.w,
                                              gt_odom->pose.pose.orientation.x,
                                              gt_odom->pose.pose.orientation.y,
                                              gt_odom->pose.pose.orientation.z);
  gtsam::Point3 position(gt_odom->pose.pose.position.x,
                         gt_odom->pose.pose.position.y,
                         gt_odom->pose.pose.position.z);
  gtsam::Vector3 velocity(gt_odom->twist.twist.linear.x,
                          gt_odom->twist.twist.linear.y,
                          gt_odom->twist.twist.linear.z);

  vio_navstate->pose_ = gtsam::Pose3(W_R_B, position);
  vio_navstate->velocity_ = velocity;
  // TODO(Toni): how can we get the ground-truth biases? For sim, ins't it 0?
  gtsam::Vector3 gyro_bias(0.0, 0.0, 0.0);
  gtsam::Vector3 acc_bias(0.0, 0.0, 0.0);
  vio_navstate->imu_bias_ = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
}

bool RosOnlineDataProvider::spin() {
  CHECK_EQ(vio_params_.camera_params_.size(), 2u);

  LOG(INFO) << "Spinning RosOnlineDataProvider.";

  // Start async spinners to get input data.
  if (!shutdown_) {
    CHECK(imu_async_spinner_);
    imu_async_spinner_->start();
    CHECK(async_spinner_);
    async_spinner_->start();
  }

  // Start our own spin to publish output data to ROS
  while (ros::ok() && !shutdown_) {
    spinOnce();  // TODO(marcus): need a sequential mode?
  }


  imu_queue_.disable();

  LOG(INFO) << "Shutting down queues ROS Async Spinner.";
  CHECK(imu_async_spinner_);
  imu_async_spinner_->stop();
  CHECK(async_spinner_);
  async_spinner_->stop();

  LOG(INFO) << "RosOnlineDataProvider successfully shutdown.";

  return false;
}

bool RosOnlineDataProvider::spinOnce() {
  // Publish frontend output at frame rate
  FrontendOutput::Ptr frame_rate_frontend_output = nullptr;
  if (frame_rate_frontend_output_queue_.pop(frame_rate_frontend_output)) {
    publishFrontendOutput(frame_rate_frontend_output);
  }

  // Publish all output at keyframe rate (backend, mesher, etc)
  publishSyncedOutputs();

  // Publish lcd output at whatever frame rate it might go
  LcdOutput::Ptr lcd_output = nullptr;
  if (lcd_output_queue_.pop(lcd_output)) {
    publishLcdOutput(lcd_output);
  }

  // ros::spinOnce(); // No need because we use an async spinner, see ctor.

  return true;
}

}  // namespace VIO
