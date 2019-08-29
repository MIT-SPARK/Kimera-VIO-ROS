/**
 * @file   ros-data-source.cpp
 * @brief  ROS wrapper for online processing.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#include "spark-vio-ros/ros-data-source.h"

#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

namespace VIO {

RosDataProvider::RosDataProvider()
    : RosBaseDataProvider(),
      left_img_subscriber_(),
      right_img_subscriber_(),
      sync_(nullptr),
      // initialize last timestamp (img) to be 0
      last_timestamp_(0),
      // initialize last timestamp (imu) to be 0
      last_imu_timestamp_(0),
      // keep track of number of frames processed
      frame_count_(1) {
  ROS_INFO("Starting SparkVIO wrapper for online");

  parseImuData(&imu_data_, &pipeline_params_.imu_params_);
  // parse backend/frontend parameters
  parseBackendParams();
  parseFrontendParams();
  // print parameters for check
  print();

  // Start IMU subscriber
  imu_subscriber_ =
      nh_imu_.subscribe("imu", 50, &RosDataProvider::callbackIMU, this);

  // Define Callback Queue for IMU Data
  ros::CallbackQueue imu_queue;
  nh_imu_.setCallbackQueue(&imu_queue);

  // Spawn Async Spinner (Running on Custom Queue) for IMU
  // 0 to use number of processor queues

  ros::AsyncSpinner async_spinner_imu(0, &imu_queue);
  async_spinner_imu.start();

  // Subscribe to stereo images. Approx time sync, should be exact though...
  DCHECK(it_);
  left_img_subscriber_.subscribe(*it_, "left_cam", 1);
  right_img_subscriber_.subscribe(*it_, "right_cam", 1);
  sync_ = VIO::make_unique<message_filters::Synchronizer<sync_pol>>(
      sync_pol(10), left_img_subscriber_, right_img_subscriber_);
  DCHECK(sync_);
  sync_->registerCallback(
      boost::bind(&RosDataProvider::callbackCamAndProcessStereo, this, _1, _2));

  // Define Callback Queue for Cam Data
  ros::CallbackQueue cam_queue;
  nh_cam_.setCallbackQueue(&cam_queue);

  // Spawn Async Spinner (Running on Custom Queue) for Cam
  ros::AsyncSpinner async_spinner_cam(0, &cam_queue);
  async_spinner_cam.start();

  ////// Define Reinitializer Subscriber
  reinit_flag_subscriber_ = nh_reinit_.subscribe(
      "reinit_flag", 10, &RosDataProvider::callbackReinit, this);
  reinit_pose_subscriber_ = nh_reinit_.subscribe(
      "reinit_pose", 10, &RosDataProvider::callbackReinitPose, this);

  // Define Callback Queue for Reinit Data
  ros::CallbackQueue reinit_queue;
  nh_reinit_.setCallbackQueue(&reinit_queue);

  ros::AsyncSpinner async_spinner_reinit(0, &reinit_queue);
  async_spinner_reinit.start();

  ROS_INFO(">>>>>>> Started data subscribers <<<<<<<<");
}

RosDataProvider::~RosDataProvider() {}

// IMU callback
void RosDataProvider::callbackIMU(const sensor_msgs::ImuConstPtr &msgIMU) {
  // Callback and store IMU data and timestamp
  // until next StereoImuSyncPacket is made
  gtsam::Vector6 gyroAccData;
  gtsam::Vector6 imu_accgyr;

  imu_accgyr(0) = msgIMU->linear_acceleration.x;
  imu_accgyr(1) = msgIMU->linear_acceleration.y;
  imu_accgyr(2) = msgIMU->linear_acceleration.z;
  imu_accgyr(3) = msgIMU->angular_velocity.x;
  imu_accgyr(4) = msgIMU->angular_velocity.y;
  imu_accgyr(5) = msgIMU->angular_velocity.z;
  // Adapt imu timestamp to account for time shift in IMU-cam
  ros::Duration imu_shift(pipeline_params_.imu_params_.imu_shift_);
  Timestamp timestamp = msgIMU->header.stamp.toNSec() - imu_shift.toNSec();
  // t_imu = t_cam + imu_shift (see: Kalibr)
  // ROS_INFO("Recieved message at time %ld", timestamp);

  // add measurement to buffer
  // time strictly increasing
  if (timestamp > last_imu_timestamp_) {
    imu_data_.imu_buffer_.addMeasurement(timestamp, imu_accgyr);
  } else {
    ROS_ERROR("Current IMU timestamp is less or equal to previous timestamp.");
  }

  if (last_timestamp_ == 0) {
    // This is the first message we receive, initialize first timestamp.
    last_timestamp_ = timestamp;
  }

  last_imu_timestamp_ = timestamp;
}

// Reinitialization callback
void RosDataProvider::callbackReinit(
    const std_msgs::Bool::ConstPtr &reinitFlag) {
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
void RosDataProvider::callbackReinitPose(
    const geometry_msgs::PoseStamped &reinitPose) {
  // Set reinitialization pose
  gtsam::Rot3 rotation(gtsam::Quaternion(
      reinitPose.pose.orientation.w, reinitPose.pose.orientation.x,
      reinitPose.pose.orientation.y, reinitPose.pose.orientation.z));

  gtsam::Point3 position(reinitPose.pose.position.x, reinitPose.pose.position.y,
                         reinitPose.pose.position.z);
  gtsam::Pose3 pose = gtsam::Pose3(rotation, position);
  reinit_packet_.setReinitPose(pose);
}

// Callback for stereo images and main spin
void RosDataProvider::callbackCamAndProcessStereo(
    const sensor_msgs::ImageConstPtr &msgLeft,
    const sensor_msgs::ImageConstPtr &msgRight) {
  // store in stereo buffer
  stereo_buffer_.addStereoFrame(msgLeft, msgRight);
}

bool RosDataProvider::spin() {
  // ros::Rate rate(60);
  while (ros::ok()) {
    // Main spin of the data provider: Interpolates IMU data and build
    // StereoImuSyncPacket (Think of this as the spin of the other
    // parser/data-providers)
    const Timestamp &timestamp = stereo_buffer_.getEarliestTimestamp();
    if (timestamp <= last_timestamp_) {
      if (stereo_buffer_.size() != 0) {
        ROS_WARN(
            "Next frame in image buffer is from the same or "
            "earlier time than the last processed frame. Skip "
            "frame.");
        // remove next frame (this would usually happen for the first
        // frame)
        stereo_buffer_.removeNext();
      }
      // else just waiting for next stereo frames
    } else {
      // Test if IMU data available
      ImuMeasurements imu_meas;

      utils::ThreadsafeImuBuffer::QueryResult imu_query =
          imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
              last_timestamp_, timestamp, &imu_meas.timestamps_,
              &imu_meas.measurements_);
      if (imu_query ==
          utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
        // data available
        sensor_msgs::ImageConstPtr left_ros_img, right_ros_img;
        CHECK(stereo_buffer_.extractLatestImages(left_ros_img, right_ros_img));

        const StereoMatchingParams &stereo_matching_params =
            frontend_params_.getStereoMatchingParams();

        // Read ROS images to cv type
        cv::Mat left_image, right_image;

        switch (stereo_matching_params.vision_sensor_type_) {
          case VisionSensorType::STEREO:
            left_image = readRosImage(left_ros_img);
            right_image = readRosImage(right_ros_img);
            break;
          case VisionSensorType::RGBD:  // just use depth to "fake
                                        // right pixel matches" apply
                                        // conversion
            left_image = readRosImage(left_ros_img);
            right_image = readRosDepthImage(right_ros_img);
            break;
          default:
            LOG(FATAL) << "vision sensor type not recognised.";
            break;
        }
        // Reset reinit flag for reinit packet
        resetReinitFlag();

        // Send input data to VIO!
        vio_callback_(StereoImuSyncPacket(
            StereoFrame(frame_count_, timestamp, left_image,
                        stereo_calib_.left_camera_info_, right_image,
                        stereo_calib_.right_camera_info_,
                        stereo_calib_.camL_Pose_camR_, stereo_matching_params),
            imu_meas.timestamps_, imu_meas.measurements_, reinit_packet_));

        last_timestamp_ = timestamp;
        frame_count_++;

      } else if (imu_query == utils::ThreadsafeImuBuffer::QueryResult::
                                  kTooFewMeasurementsAvailable) {
        ROS_WARN(
            "Too few IMU measurements between next frame and "
            "last frame. Skip frame.");
        // remove next frame (this would usually for the first
        // frame)
        stereo_buffer_.removeNext();
      } else if (imu_query == utils::ThreadsafeImuBuffer::QueryResult::
                                  kDataNotYetAvailable) {
        // kNotYetAvailable then just wait for next loop...
        ROS_WARN_THROTTLE(100, "IMU data not yet available. Skip frame.");
      }
    }

    // Publish VIO output if any.
    SpinOutputPacket vio_output;
    if (vio_output_queue_.pop(vio_output)) {
      publishOutput(vio_output);
    }

    // spin loop
    ros::spinOnce();
  }

  ROS_INFO("Ros data source spin done. Shutting down vio output queue.");
  vio_output_queue_.shutdown();

  return true;
}

void RosDataProvider::print() const {
  LOG(INFO) << std::string(80, '=') << '\n'
            << ">>>>>>>>> RosDataProvider::print <<<<<<<<<<<" << '\n'
            << "camL_Pose_camR_: " << stereo_calib_.camL_Pose_camR_;
  // For each of the 2 cameras.
  LOG(INFO) << "- Left camera params:";
  stereo_calib_.left_camera_info_.print();
  LOG(INFO) << "- Right camera params:";
  stereo_calib_.right_camera_info_.print();
  LOG(INFO) << "- IMU info: ";
  imu_data_.print();
  LOG(INFO) << '\n' << std::string(80, '=');
}

}  // namespace VIO
