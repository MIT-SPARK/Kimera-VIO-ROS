/**
 * @file   RosOnlineDataProvider.cpp
 * @brief  ROS wrapper for online processing.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/RosOnlineDataProvider.h"

#include <string>
#include <vector>

#include <glog/logging.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosOnlineDataProvider::RosOnlineDataProvider(const VioParams& vio_params)
    : RosDataProviderInterface(vio_params),
      it_(nullptr),
      frame_count_(FrameId(0)),
      left_img_subscriber_(),
      right_img_subscriber_(),
      left_cam_info_subscriber_(),
      right_cam_info_subscriber_(),
      sync_img_(),
      sync_cam_info_(),
      imu_subscriber_(),
      gt_odom_subscriber_(),
      reinit_flag_subscriber_(),
      reinit_pose_subscriber_(),
      imu_queue_(),
      imu_async_spinner_(nullptr),
      async_spinner_(nullptr) {
  // Wait until time is non-zero and valid: this is because at the ctor level
  // we will be querying for gt pose and/or camera info.
  while (ros::ok() && !ros::Time::now().isValid()) {
    if (ros::Time::isSimTime()) {
      LOG_FIRST_N(INFO, 1)
          << "Waiting for ROS time to be valid... \n"
          << "(Sim Time is enabled; run rosbag with --clock argument)";
    } else {
      LOG_FIRST_N(INFO, 1) << "Waiting for ROS time to be valid...";
    }
  }

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

    // We wait for the gt pose.
    LOG(WARNING) << "Waiting for ground-truth pose to initialize VIO "
                 << "on ros topic: " << gt_odom_subscriber_.getTopic().c_str();
    static const ros::Duration kMaxTimeSecsForGtPose(3.0);
    ros::Time start = ros::Time::now();
    ros::Time current = ros::Time::now();
    while (!gt_init_pose_received_ &&
           (current - start) < kMaxTimeSecsForGtPose) {
      if (nh_.ok() && ros::ok() && !ros::isShuttingDown() && !shutdown_) {
        ros::spinOnce();
      } else {
        LOG(FATAL) << "Ros is not ok... Shutting down.";
      }
      current = ros::Time::now();
      CHECK(current.isValid());
    }

    if (!gt_init_pose_received_) {
      LOG(ERROR)
          << "Missing ground-truth pose while trying for "
          << (current - start).toSec() << " seconds.\n"
          << "Enabling autoInitialize and continuing without ground-truth "
             "pose.";
      vio_params_.backend_params_->autoInitialize_ = true;
    }
  }

  // Determine whether to use camera info topics for camera parameters:
  bool use_online_cam_params = false;
  CHECK(nh_private_.getParam("use_online_cam_params", use_online_cam_params));
  if (use_online_cam_params) {
    LOG(WARNING)
        << "Using online camera parameters instead of YAML parameter files.";

    static constexpr size_t kMaxCamInfoQueueSize = 10u;
    static constexpr size_t kMaxCamInfoSynchronizerQueueSize = 10u;
    left_cam_info_subscriber_.subscribe(
        nh_, "left_cam/camera_info", kMaxCamInfoQueueSize);
    right_cam_info_subscriber_.subscribe(
        nh_, "right_cam/camera_info", kMaxCamInfoQueueSize);

    sync_cam_info_ =
        VIO::make_unique<message_filters::Synchronizer<sync_pol_info>>(
            sync_pol_info(kMaxCamInfoSynchronizerQueueSize),
            left_cam_info_subscriber_,
            right_cam_info_subscriber_);

    DCHECK(sync_cam_info_);
    sync_cam_info_->registerCallback(
        boost::bind(&RosOnlineDataProvider::callbackCameraInfo, this, _1, _2));

    // Wait for camera info to be received.
    static const ros::Duration kMaxTimeSecsForCamInfo(10.0);
    ros::Time start = ros::Time::now();
    ros::Time current = ros::Time::now();
    while (!camera_info_received_ &&
           (current - start) < kMaxTimeSecsForCamInfo) {
      if (nh_.ok() && ros::ok() && !ros::isShuttingDown() && !shutdown_) {
        ros::spinOnce();
      } else {
        LOG(FATAL) << "Ros is not ok... Shutting down.";
      }
      current = ros::Time::now();
      CHECK(current.isValid());
    }
    LOG_IF(FATAL, !camera_info_received_)
        << "Missing camera info, while trying for " << (current - start).toSec()
        << " seconds.\n"
        << "Expected camera info in topics:\n"
        << " - Left cam info topic: " << left_cam_info_subscriber_.getTopic()
        << '\n'
        << " - Right cam info topic: " << right_cam_info_subscriber_.getTopic();
  }

  //! IMU Subscription
  // Create a dedicated queue for the Imu callback so that we can use an async
  // spinner on it to process the data lighting fast.
  static constexpr size_t kMaxImuQueueSize = 1000u;
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

  //! Vision Subscription
  // Subscribe to stereo images. Approx time sync, should be exact though...
  // We set the queue to only 1, since we prefer to drop messages to reach
  // real-time than to be delayed...
  static constexpr size_t kMaxImagesQueueSize = 1u;
  it_ = VIO::make_unique<image_transport::ImageTransport>(nh_);
  left_img_subscriber_.subscribe(
      *it_, "left_cam/image_raw", kMaxImagesQueueSize);
  right_img_subscriber_.subscribe(
      *it_, "right_cam/image_raw", kMaxImagesQueueSize);
  static constexpr size_t kMaxImageSynchronizerQueueSize = 10u;
  sync_img_ = VIO::make_unique<message_filters::Synchronizer<sync_pol_img>>(
      sync_pol_img(kMaxImageSynchronizerQueueSize),
      left_img_subscriber_,
      right_img_subscriber_);

  DCHECK(sync_img_);
  sync_img_->registerCallback(
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

  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(nh_private_.getParam("left_cam_frame_id", left_cam_frame_id_));
  CHECK(!left_cam_frame_id_.empty());
  CHECK(nh_private_.getParam("right_cam_frame_id", right_cam_frame_id_));
  CHECK(!right_cam_frame_id_.empty());

  publishStaticTf(vio_params_.camera_params_.at(0).body_Pose_cam_,
                  base_link_frame_id_,
                  left_cam_frame_id_);
  publishStaticTf(vio_params_.camera_params_.at(1).body_Pose_cam_,
                  base_link_frame_id_,
                  right_cam_frame_id_);

  //! IMU Spinner
  if (vio_params_.parallel_run_) {
    // Imu Async Spinner: will process the imu queue only, instead of ROS'
    // global
    // queue. A value of 0 means to use the number of processor cores.
    static constexpr size_t kImuSpinnerThreads = 2u;
    imu_async_spinner_ =
        VIO::make_unique<ros::AsyncSpinner>(kImuSpinnerThreads, &imu_queue_);

    //! Vision Spinner
    // This async spinner will process the regular Global callback queue of ROS.
    static constexpr size_t kGlobalSpinnerThreads = 2u;
    async_spinner_ = VIO::make_unique<ros::AsyncSpinner>(kGlobalSpinnerThreads);
  } else {
    LOG(INFO) << "RosOnlineDataProvider running in sequential mode.";
  }
}

RosOnlineDataProvider::~RosOnlineDataProvider() {
  VLOG(1) << "RosOnlineDataProvider destructor called.";
  imu_queue_.disable();

  if (imu_async_spinner_) imu_async_spinner_->stop();
  if (async_spinner_) async_spinner_->stop();

  LOG(INFO) << "RosOnlineDataProvider successfully shutdown.";
}

bool RosOnlineDataProvider::spin() {
  if (!shutdown_) {
    if (vio_params_.parallel_run_) {
      return parallelSpin();
    } else {
      return sequentialSpin();
    }
  } else {
    return false;
  }
}

bool RosOnlineDataProvider::parallelSpin() {
  CHECK(vio_params_.parallel_run_);
  // Start async spinners to get input data (only once!).
  if (!started_async_spinners_) {
    VLOG(10) << "Starting Async spinners.";
    CHECK(imu_async_spinner_);
    imu_async_spinner_->start();
    CHECK(async_spinner_);
    async_spinner_->start();
    started_async_spinners_ = true;
  } else {
    VLOG(10) << "Async spinners already started.";
  }
  return true;
}

bool RosOnlineDataProvider::sequentialSpin() {
  CHECK(!vio_params_.parallel_run_)
      << "This should be only running if we are in sequential mode!";
  CHECK(!imu_async_spinner_) << "There should not be an IMU async spinner "
                                "constructed if in sequential mode.";
  CHECK(!async_spinner_) << "There should not be a general async spinner "
                            "constructed if in sequential mode.";
  CHECK(imu_queue_.isEnabled());
  // First call callbacks in our custom IMU queue.
  imu_queue_.callAvailable(ros::WallDuration(0.1));
  // Then call the rest of callbacks.
  // which is the same as:
  // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
  ros::spinOnce();
  return true;
}

// TODO(marcus): with the readRosImage, this is a slow callback. Might be too
// slow...
void RosOnlineDataProvider::callbackStereoImages(
    const sensor_msgs::ImageConstPtr& left_msg,
    const sensor_msgs::ImageConstPtr& right_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 2u);
  const CameraParams& left_cam_info = vio_params_.camera_params_.at(0);
  const CameraParams& right_cam_info = vio_params_.camera_params_.at(1);

  CHECK(left_msg);
  CHECK(right_msg);
  const Timestamp& timestamp_left = left_msg->header.stamp.toNSec();
  const Timestamp& timestamp_right = right_msg->header.stamp.toNSec();

  if (!shutdown_) {
    CHECK(left_frame_callback_)
        << "Did you forget to register the left frame callback?";
    left_frame_callback_(VIO::make_unique<Frame>(
        frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));

    if (vio_params_.frontend_type_ == VIO::FrontendType::kStereoImu) {
      CHECK(right_frame_callback_)
          << "Did you forget to register the right frame callback?";
      right_frame_callback_(VIO::make_unique<Frame>(frame_count_,
                                                    timestamp_right,
                                                    right_cam_info,
                                                    readRosImage(right_msg)));
    }
    frame_count_++;
  }
}

void RosOnlineDataProvider::callbackCameraInfo(
    const sensor_msgs::CameraInfoConstPtr& left_msg,
    const sensor_msgs::CameraInfoConstPtr& right_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 2u);

  // Initialize CameraParams for pipeline.
  utils::msgCamInfoToCameraParams(left_msg,
                                  base_link_frame_id_,
                                  left_cam_frame_id_,
                                  &vio_params_.camera_params_.at(0));
  utils::msgCamInfoToCameraParams(right_msg,
                                  base_link_frame_id_,
                                  right_cam_frame_id_,
                                  &vio_params_.camera_params_.at(1));

  vio_params_.camera_params_.at(0).print();
  vio_params_.camera_params_.at(1).print();

  // Unregister this callback as it is no longer needed.
  LOG(INFO)
      << "Unregistering CameraInfo subscribers as data has been received.";
  left_cam_info_subscriber_.unsubscribe();
  right_cam_info_subscriber_.unsubscribe();

  // Signal the correct reception of camera info
  camera_info_received_ = true;
}

void RosOnlineDataProvider::callbackIMU(
    const sensor_msgs::ImuConstPtr& imu_msg) {
  // TODO(TONI): detect jump backwards in time?

  VIO::ImuAccGyr imu_accgyr;

  imu_accgyr(0) = imu_msg->linear_acceleration.x;
  imu_accgyr(1) = imu_msg->linear_acceleration.y;
  imu_accgyr(2) = imu_msg->linear_acceleration.z;
  imu_accgyr(3) = imu_msg->angular_velocity.x;
  imu_accgyr(4) = imu_msg->angular_velocity.y;
  imu_accgyr(5) = imu_msg->angular_velocity.z;

  // Adapt imu timestamp to account for time shift in IMU-cam
  Timestamp timestamp = imu_msg->header.stamp.toNSec();

  const ros::Duration imu_shift(vio_params_.imu_params_.imu_time_shift_);
  if (imu_shift != ros::Duration(0)) {
    LOG_EVERY_N(WARNING, 1000) << "imu_shift is not 0.";
    timestamp -= imu_shift.toNSec();
  }

  if (!shutdown_) {
    CHECK(imu_single_callback_)
        << "Did you forget to register the IMU callback?";
    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
  }
}

// Ground-truth odometry callback
void RosOnlineDataProvider::callbackGtOdomOnce(
    const nav_msgs::Odometry::ConstPtr& gt_odom_msg) {
  LOG(WARNING) << "Using initial ground-truth state for initialization.";
  CHECK(gt_odom_msg);
  utils::rosOdometryToVioNavState(
      *gt_odom_msg,
      nh_private_,
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

void RosOnlineDataProvider::publishStaticTf(const gtsam::Pose3& pose,
                                            const std::string& parent_frame_id,
                                            const std::string& child_frame_id) {
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transform_stamped;
  // TODO(Toni): Warning: using ros::Time::now(), will that bring issues?
  static_transform_stamped.header.stamp = ros::Time::now();
  static_transform_stamped.header.frame_id = parent_frame_id;
  static_transform_stamped.child_frame_id = child_frame_id;
  utils::gtsamPoseToRosTf(pose, &static_transform_stamped.transform);
  static_broadcaster.sendTransform(static_transform_stamped);
}

}  // namespace VIO
