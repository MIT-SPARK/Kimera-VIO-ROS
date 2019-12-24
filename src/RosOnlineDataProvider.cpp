/**
 * @file   ros-data-source.cpp
 * @brief  ROS wrapper for online processing.
 * @author Yun Chang
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

RosOnlineDataProvider::RosOnlineDataProvider()
    : RosDataProviderInterface(),
      frame_count_(FrameId(0)),
      left_img_subscriber_(),
      right_img_subscriber_(),
      imu_subscriber_(),
      reinit_flag_subscriber_(),
      reinit_pose_subscriber_(),
      vio_queue_(),
      async_spinner_(nullptr) {
  ROS_INFO("Starting KimeraVIO wrapper for online");

  // Start IMU subscriber
  static constexpr size_t kMaxImuQueueSize = 50u;
  imu_subscriber_ = nh_.subscribe(
      "imu", kMaxImuQueueSize, &RosOnlineDataProvider::callbackIMU, this);

  // Subscribe to stereo images. Approx time sync, should be exact though...
  static constexpr size_t kMaxImagesQueueSize = 1u;
  CHECK(it_);
  left_img_subscriber_.subscribe(*it_, "left_cam", kMaxImagesQueueSize);
  right_img_subscriber_.subscribe(*it_, "right_cam", kMaxImagesQueueSize);
  static constexpr size_t kMaxImageSynchronizerQueueSize = 10u;
  sync_ = VIO::make_unique<message_filters::Synchronizer<sync_pol>>(
      sync_pol(kMaxImageSynchronizerQueueSize),
      left_img_subscriber_,
      right_img_subscriber_);
  DCHECK(sync_);
  sync_->registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackStereoImages, this, _1, _2));

  ////// Define Reinitializer Subscriber
  static constexpr size_t kMaxReinitQueueSize = 50u;
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

  // Define Callback Queue separate from Global Callback Queue for faster
  // processing.
  //nh_.setCallbackQueue(&vio_queue_);

  //ros::AsyncSpinner async_spinner(kSpinnerThreads, &vio_queue_);
  static constexpr size_t kSpinnerThreads = 0;
  async_spinner_ = VIO::make_unique<ros::AsyncSpinner>(kSpinnerThreads);
  async_spinner_->start();

  ROS_INFO(">>>>>>> Started data subscribers <<<<<<<<");
}

RosOnlineDataProvider::~RosOnlineDataProvider() {
  LOG(INFO) << "RosDataProvider destructor called.";
}

// TODO(marcus): with the readRosImage, this is a slow callback. Might be too
// slow...
void RosOnlineDataProvider::callbackStereoImages(
    const sensor_msgs::ImageConstPtr& left_msg,
    const sensor_msgs::ImageConstPtr& right_msg) {
  static const CameraParams& left_cam_info =
      pipeline_params_.camera_params_.at(0);
  static const CameraParams& right_cam_info =
      pipeline_params_.camera_params_.at(1);

  const Timestamp& timestamp_left = left_msg->header.stamp.toNSec();
  const Timestamp& timestamp_right = right_msg->header.stamp.toNSec();

  CHECK(left_frame_callback_)
      << "Did you forget to register the left frame callback?";
  CHECK(right_frame_callback_)
      << "Did you forget to register the right frame callback?";

  left_frame_callback_(VIO::make_unique<Frame>(
      frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));
  right_frame_callback_(VIO::make_unique<Frame>(
      frame_count_, timestamp_right, right_cam_info, readRosImage(right_msg)));

  frame_count_++;
}

void RosOnlineDataProvider::callbackIMU(
    const sensor_msgs::ImuConstPtr& msgIMU) {
  VIO::ImuAccGyr imu_accgyr;

  imu_accgyr(0) = msgIMU->linear_acceleration.x;
  imu_accgyr(1) = msgIMU->linear_acceleration.y;
  imu_accgyr(2) = msgIMU->linear_acceleration.z;
  imu_accgyr(3) = msgIMU->angular_velocity.x;
  imu_accgyr(4) = msgIMU->angular_velocity.y;
  imu_accgyr(5) = msgIMU->angular_velocity.z;

  // Adapt imu timestamp to account for time shift in IMU-cam
  Timestamp timestamp = msgIMU->header.stamp.toNSec();

  static const ros::Duration imu_shift(pipeline_params_.imu_params_.imu_shift_);
  if (imu_shift != ros::Duration(0)) {
    LOG_EVERY_N(WARNING, 1000) << "imu_shift is not 0.";
    timestamp -= imu_shift.toNSec();
  }

  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
  imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
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

bool RosOnlineDataProvider::spin() {
  CHECK_EQ(pipeline_params_.camera_params_.size(), 2u);

  while (ros::ok()) {
    spinOnce();  // TODO(marcus): need a sequential mode?
  }

  ROS_INFO("Ros data source spin done. Shutting down queues.");
  backend_output_queue_.shutdown();
  frontend_output_queue_.shutdown();
  mesher_output_queue_.shutdown();
  lcd_output_queue_.shutdown();

  ROS_INFO("Shutting down queues ROS Async Spinner.");
  CHECK(async_spinner_);
  async_spinner_->stop();

  return false;
}

bool RosOnlineDataProvider::spinOnce() {
  // Publish VIO output if any.
  publishSyncedOutputs();

  // Publish LCD output if any.
  LcdOutput::Ptr lcd_output = nullptr;
  if (lcd_output_queue_.pop(lcd_output)) {
    publishLcdOutput(lcd_output);
  }

  // ros::spinOnce();

  return true;
}

}  // namespace VIO
