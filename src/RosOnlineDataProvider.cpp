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
      sync_img_(),
      imu_subscriber_(),
      gt_odom_subscriber_(),
      external_odom_subscriber_(),
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
  static constexpr size_t kMaxGtOdomQueueSize = 10u;
  if (vio_params_.backend_params_->autoInitialize_ == 0 || log_gt_data_) {
    gt_odom_subscriber_ = nh_.subscribe("gt_odom",
                                        kMaxGtOdomQueueSize,
                                        &RosOnlineDataProvider::callbackGtOdom,
                                        this);
  }

  if (vio_params_.backend_params_->autoInitialize_ == 0) {
    LOG(INFO) << "Requested initialization from ground-truth. "
              << "Initializing ground-truth odometry one-shot subscriber.";
    // We wait for the gt pose.
    LOG(WARNING) << "Waiting for ground-truth pose to initialize VIO "
                 << "on ros topic: " << gt_odom_subscriber_.getTopic().c_str();

    double gt_pose_wait_time_s = 10.0;
    nh_private_.getParam("gt_pose_wait_time_s", gt_pose_wait_time_s);
    const ros::Duration kMaxTimeSecsForGtPose(gt_pose_wait_time_s);

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
  } else {
    // disable message about using gt odom for init when only logging gt odom
    gt_init_pose_received_ = true;
  }

  // decide whether or not to force timestamp synchronization
  nh_private_.getParam("force_same_image_timestamp",
                       force_same_image_timestamp_);

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
  it_ = std::make_unique<image_transport::ImageTransport>(nh_);
  switch (vio_params_.frontend_type_) {
    case FrontendType::kMonoImu: {
      subscribeMono(kMaxImagesQueueSize);
      break;
    }
    case FrontendType::kStereoImu: {
      subscribeStereo(kMaxImagesQueueSize);
      break;
    }
    case FrontendType::kRgbdImu: {
      subscribeRgbd(kMaxImagesQueueSize);
      break;
    }

    default: { LOG(FATAL) << "Frontend type not recognized."; }
  }

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

  // External Odometry Subscription
  CHECK(nh_private_.getParam("use_external_odom", use_external_odom_));
  if (use_external_odom_) {
    static constexpr size_t kMaxExternalOdomQueueSize = 1000u;
    external_odom_subscriber_ =
        nh_.subscribe("external_odom",
                      kMaxExternalOdomQueueSize,
                      &RosOnlineDataProvider::callbackExternalOdom,
                      this);
  }

  publishStaticTf(vio_params_.camera_params_.at(0).body_Pose_cam_,
                  base_link_frame_id_,
                  left_cam_frame_id_);
  if (vio_params_.camera_params_.size() == 2) {
    publishStaticTf(vio_params_.camera_params_.at(1).body_Pose_cam_,
                    base_link_frame_id_,
                    right_cam_frame_id_);
  }

  //! IMU Spinner
  if (vio_params_.parallel_run_) {
    // Imu Async Spinner: will process the imu queue only, instead of ROS'
    // global
    // queue. A value of 0 means to use the number of processor cores.
    static constexpr size_t kImuSpinnerThreads = 2u;
    imu_async_spinner_ =
        std::make_unique<ros::AsyncSpinner>(kImuSpinnerThreads, &imu_queue_);

    //! Vision Spinner
    // This async spinner will process the regular Global callback queue of ROS.
    static constexpr size_t kGlobalSpinnerThreads = 2u;
    async_spinner_ = std::make_unique<ros::AsyncSpinner>(kGlobalSpinnerThreads);
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

void RosOnlineDataProvider::subscribeMono(const size_t& kMaxImagesQueueSize) {
  left_img_subscriber_.subscribe(
      *it_, "left_cam/image_raw", kMaxImagesQueueSize);
  left_img_subscriber_.registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackMonoImage, this, _1));
}

void RosOnlineDataProvider::subscribeStereo(const size_t& kMaxImagesQueueSize) {
  left_img_subscriber_.subscribe(
      *it_, "left_cam/image_raw", kMaxImagesQueueSize);
  right_img_subscriber_.subscribe(
      *it_, "right_cam/image_raw", kMaxImagesQueueSize);
  static constexpr size_t kMaxImageSynchronizerQueueSize = 10u;
  sync_img_ = std::make_unique<message_filters::Synchronizer<sync_pol_img>>(
      sync_pol_img(kMaxImageSynchronizerQueueSize),
      left_img_subscriber_,
      right_img_subscriber_);

  DCHECK(sync_img_);
  sync_img_->registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackStereoImages, this, _1, _2));
}

void RosOnlineDataProvider::subscribeRgbd(const size_t& kMaxImagesQueueSize) {
  left_img_subscriber_.subscribe(
      *it_, "left_cam/image_raw", kMaxImagesQueueSize);
  depth_img_subscriber_.subscribe(
      *it_,
      "depth_cam/image_raw",
      kMaxImagesQueueSize,
      image_transport::TransportHints(
          "raw", ros::TransportHints(), nh_private_, "image_transport_depth"));
  static constexpr size_t kMaxImageSynchronizerQueueSize = 10u;
  sync_img_ = std::make_unique<message_filters::Synchronizer<sync_pol_img>>(
      sync_pol_img(kMaxImageSynchronizerQueueSize),
      left_img_subscriber_,
      depth_img_subscriber_);

  DCHECK(sync_img_);
  sync_img_->registerCallback(
      boost::bind(&RosOnlineDataProvider::callbackRgbdImages, this, _1, _2));
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
void RosOnlineDataProvider::callbackMonoImage(
    const sensor_msgs::ImageConstPtr& img_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 1u);
  const CameraParams& cam_info = vio_params_.camera_params_.at(0);

  CHECK(img_msg);
  const Timestamp& timestamp = img_msg->header.stamp.toNSec();

  if (!shutdown_) {
    CHECK(left_frame_callback_)
        << "Did you forget to register the left frame callback?";
    left_frame_callback_(std::make_unique<Frame>(
        frame_count_, timestamp, cam_info, readRosImage(img_msg)));
    frame_count_++;
  }
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
    left_frame_callback_(std::make_unique<Frame>(
        frame_count_, timestamp_left, left_cam_info, readRosImage(left_msg)));

    if (vio_params_.frontend_type_ == VIO::FrontendType::kStereoImu) {
      CHECK(right_frame_callback_)
          << "Did you forget to register the right frame callback?";
      right_frame_callback_(std::make_unique<Frame>(
          frame_count_,
          force_same_image_timestamp_ ? timestamp_left : timestamp_right,
          right_cam_info,
          readRosImage(right_msg)));
    }
    frame_count_++;
  }
}

void RosOnlineDataProvider::callbackRgbdImages(
    const sensor_msgs::ImageConstPtr& color_msg,
    const sensor_msgs::ImageConstPtr& depth_msg) {
  CHECK_GE(vio_params_.camera_params_.size(), 1u);
  const CameraParams& cam_info = vio_params_.camera_params_.at(0);

  CHECK(color_msg);
  CHECK(depth_msg);
  const Timestamp& timestamp_color = color_msg->header.stamp.toNSec();
  const Timestamp& timestamp_depth = depth_msg->header.stamp.toNSec();

  if (!shutdown_) {
    CHECK(left_frame_callback_)
        << "Did you forget to register the color frame callback?";
    left_frame_callback_(std::make_unique<Frame>(
        frame_count_, timestamp_color, cam_info, readRosImage(color_msg)));

    CHECK(depth_frame_callback_)
        << "Did you forget to register the depth frame callback?";
    depth_frame_callback_(std::make_unique<DepthFrame>(
        frame_count_,
        force_same_image_timestamp_ ? timestamp_color : timestamp_depth,
        readRosDepthImage(depth_msg)));
  }

  frame_count_++;
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

  if (!shutdown_) {
    CHECK(imu_single_callback_)
        << "Did you forget to register the IMU callback?";
    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
  }
}

// Ground-truth odometry callback
void RosOnlineDataProvider::callbackGtOdom(
    const nav_msgs::Odometry::ConstPtr& gt_odom_msg) {
  CHECK(gt_odom_msg);
  if (!gt_init_pose_received_) {
    LOG(WARNING) << "Using initial ground-truth state for initialization.";
    utils::rosOdometryToVioNavState(
        *gt_odom_msg,
        nh_private_,
        &vio_params_.backend_params_->initial_ground_truth_state_);

    // Signal receptance of ground-truth pose.
    gt_init_pose_received_ = true;
  }

  CHECK(gt_init_pose_received_);
  if (log_gt_data_) {
    logGtData(gt_odom_msg);
  } else {
    // Shutdown to prevent more than one message being processed.
    gt_odom_subscriber_.shutdown();
  }
}

void RosOnlineDataProvider::callbackExternalOdom(
    const nav_msgs::Odometry::ConstPtr& odom_msg) {
  CHECK(odom_msg);
  VIO::VioNavState kimera_odom;
  utils::rosOdometryToVioNavState(*odom_msg, nh_private_, &kimera_odom);
  external_odom_callback_(ExternalOdomMeasurement(
      odom_msg->header.stamp.toNSec(),
      gtsam::NavState(kimera_odom.pose_, kimera_odom.velocity_)));
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
