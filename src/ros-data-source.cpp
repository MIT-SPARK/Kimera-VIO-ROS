/**
 * @file   ros-data-source.cpp
 * @brief  ROS wrapper for online processing.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#include "kimera-ros/ros-data-source.h"

#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

namespace VIO {

RosDataProvider::RosDataProvider()
    : RosBaseDataProvider(),
      left_img_sub_(),
      right_img_sub_(),
      frame_count_left_(FrameId(0)),
      frame_count_right_(FrameId(0)),
      imu_input_queue_("IMU Input Queue"),
      left_camera_input_queue_("Left Camera Input Queue"),
      right_camera_input_queue_("Right Camera Input Queue") {
  ROS_INFO("Starting KimeraVIO wrapper for online");

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

  // Subscribe to stereo images.
  DCHECK(it_);
  left_img_sub_ =
      it_->subscribe("left_cam", 1, &RosDataProvider::callbackLeftImage, this);

  right_img_sub_ = it_->subscribe(
      "right_cam", 1, &RosDataProvider::callbackRightImage, this);

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

RosDataProvider::~RosDataProvider() {
  LOG(INFO) << "RosDataProvider destructor called.";
}

void RosDataProvider::callbackLeftImage(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Pushing left image to queue.");
  left_camera_input_queue_.push(msg);
}

void RosDataProvider::callbackRightImage(
    const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Pushing right image to queue.");
  right_camera_input_queue_.push(msg);
}

void RosDataProvider::callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU) {
  imu_input_queue_.push(msgIMU);
  // // Callback and store IMU data and timestamp
  // // until next StereoImuSyncPacket is made
  // gtsam::Vector6 gyroAccData;
  // gtsam::Vector6 imu_accgyr;

  // imu_accgyr(0) = msgIMU->linear_acceleration.x;
  // imu_accgyr(1) = msgIMU->linear_acceleration.y;
  // imu_accgyr(2) = msgIMU->linear_acceleration.z;
  // imu_accgyr(3) = msgIMU->angular_velocity.x;
  // imu_accgyr(4) = msgIMU->angular_velocity.y;
  // imu_accgyr(5) = msgIMU->angular_velocity.z;
  // // Adapt imu timestamp to account for time shift in IMU-cam
  // static const ros::Duration
  // imu_shift(pipeline_params_.imu_params_.imu_shift_); Timestamp timestamp =
  // msgIMU->header.stamp.toNSec() - imu_shift.toNSec();
  // // t_imu = t_cam + imu_shift (see: Kalibr)
  // // ROS_INFO("Recieved message at time %ld", timestamp);

  // // add measurement to buffer
  // // time strictly increasing
  // if (timestamp > last_imu_timestamp_) {
  //   imu_data_.imu_buffer_.addMeasurement(timestamp, imu_accgyr);
  // } else {
  //   ROS_ERROR("Current IMU timestamp is less or equal to previous
  //   timestamp.");
  // }

  // if (last_timestamp_ == 0) {
  //   // This is the first message we receive, initialize first timestamp.
  //   last_timestamp_ = timestamp;
  // }

  // last_imu_timestamp_ = timestamp;
}

// Reinitialization callback
void RosDataProvider::callbackReinit(
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
void RosDataProvider::callbackReinitPose(
    const geometry_msgs::PoseStamped& reinitPose) {
  // Set reinitialization pose
  gtsam::Rot3 rotation(gtsam::Quaternion(reinitPose.pose.orientation.w,
                                         reinitPose.pose.orientation.x,
                                         reinitPose.pose.orientation.y,
                                         reinitPose.pose.orientation.z));

  gtsam::Point3 position(reinitPose.pose.position.x,
                         reinitPose.pose.position.y,
                         reinitPose.pose.position.z);
  gtsam::Pose3 pose = gtsam::Pose3(rotation, position);
  reinit_packet_.setReinitPose(pose);
}

bool RosDataProvider::spin() {
  CHECK_EQ(pipeline_params_.camera_params_.size(), 2u);

  while (ros::ok()) {
    spinOnce();  // TODO(marcus): need a sequential mode?
  }

  ROS_INFO("Ros data source spin done. Shutting down queues.");
  imu_input_queue_.shutdown();
  left_camera_input_queue_.shutdown();
  right_camera_input_queue_.shutdown();
  backend_output_queue_.shutdown();
  frontend_output_queue_.shutdown();
  mesher_output_queue_.shutdown();
  lcd_output_queue_.shutdown();

  return false;
}

bool RosDataProvider::spinOnce() {
  static const CameraParams& left_cam_info =
      pipeline_params_.camera_params_.at(0);
  static const CameraParams& right_cam_info =
      pipeline_params_.camera_params_.at(1);

  // Send data to callbacks as needed
  sensor_msgs::ImuConstPtr imu_data;
  if (imu_input_queue_.pop(imu_data)) {
    CHECK_NOTNULL(imu_data);
    ROS_INFO("Pushing imu_data to pipeline.");

    ImuAccGyr imu_accgyr;

    imu_accgyr(0) = imu_data->linear_acceleration.x;
    imu_accgyr(1) = imu_data->linear_acceleration.y;
    imu_accgyr(2) = imu_data->linear_acceleration.z;
    imu_accgyr(3) = imu_data->angular_velocity.x;
    imu_accgyr(4) = imu_data->angular_velocity.y;
    imu_accgyr(5) = imu_data->angular_velocity.z;

    // Adapt imu timestamp to account for time shift in IMU-cam
    static const ros::Duration imu_shift(
        pipeline_params_.imu_params_.imu_shift_);
    const Timestamp& timestamp =
        imu_data->header.stamp.toNSec() - imu_shift.toNSec();

    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
  }

  sensor_msgs::ImageConstPtr left_img;
  if (left_camera_input_queue_.pop(left_img)) {
    CHECK_NOTNULL(left_img);
    ROS_INFO("Pushing left frame to pipeline.");

    const Timestamp& timestamp = left_img->header.stamp.toNSec();

    left_frame_callback_(VIO::make_unique<Frame>(
        frame_count_left_, timestamp, left_cam_info, readRosImage(left_img)));
    frame_count_left_++;
  }

  sensor_msgs::ImageConstPtr right_img;
  if (right_camera_input_queue_.pop(right_img)) {
    CHECK_NOTNULL(right_img);
    ROS_INFO("Pushing right frame to pipeline.");

    const Timestamp& timestamp = right_img->header.stamp.toNSec();

    right_frame_callback_(VIO::make_unique<Frame>(frame_count_right_,
                                                  timestamp,
                                                  right_cam_info,
                                                  readRosImage(right_img)));
    frame_count_right_++;
  }

  // Publish VIO output if any.
  FrontendOutput::Ptr frontend_output;
  BackendOutput::Ptr backend_output;
  MesherOutput::Ptr mesher_output;
  if (getVioOutput(frontend_output, backend_output, mesher_output)) {
    publishVioOutput(frontend_output, backend_output, mesher_output);
  }

  // Publish LCD output if any.
  LcdOutput::Ptr lcd_output = nullptr;
  if (lcd_output_queue_.pop(lcd_output)) {
    publishLcdOutput(lcd_output);
  }

  ros::spinOnce();

  return true;
}

// bool RosDataProvider::spin() {
//   // ros::Rate rate(60);
//   while (ros::ok()) {
//     // Main spin of the data provider: Interpolates IMU data and build
//     // StereoImuSyncPacket (Think of this as the spin of the other
//     // parser/data-providers)
//     const Timestamp& timestamp = stereo_buffer_.getEarliestTimestamp();
//     if (timestamp <= last_timestamp_) {
//       if (stereo_buffer_.size() != 0) {
//         ROS_WARN(
//             "Next frame in image buffer is from the same or "
//             "earlier time than the last processed frame. Skip "
//             "frame.");
//         // remove next frame (this would usually happen for the first
//         // frame)
//         stereo_buffer_.removeNext();
//       }
//       // else just waiting for next stereo frames
//     } else {
//       // Test if IMU data available
//       ImuMeasurements imu_meas;

//       utils::ThreadsafeImuBuffer::QueryResult imu_query =
//           imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
//               last_timestamp_,
//               timestamp,
//               &imu_meas.timestamps_,
//               &imu_meas.measurements_);
//       if (imu_query ==
//           utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
//         // data available
//         sensor_msgs::ImageConstPtr left_ros_img, right_ros_img;
//         CHECK(stereo_buffer_.extractLatestImages(left_ros_img,
//         right_ros_img));

//         const StereoMatchingParams& stereo_matching_params =
//             frontend_params_.getStereoMatchingParams();

//         // Read ROS images to cv type
//         cv::Mat left_image, right_image;

//         switch (stereo_matching_params.vision_sensor_type_) {
//           case VisionSensorType::STEREO:
//             left_image = readRosImage(left_ros_img);
//             right_image = readRosImage(right_ros_img);
//             break;
//           case VisionSensorType::RGBD:  // just use depth to "fake
//                                         // right pixel matches" apply
//                                         // conversion
//             left_image = readRosImage(left_ros_img);
//             right_image = readRosDepthImage(right_ros_img);
//             break;
//           default:
//             LOG(FATAL) << "vision sensor type not recognised.";
//             break;
//         }
//         // Reset reinit flag for reinit packet
//         resetReinitFlag();

//         // Send input data to VIO!
//         vio_callback_(VIO::make_unique<StereoImuSyncPacket>(
//             StereoFrame(frame_count_,
//                         timestamp,
//                         left_image,
//                         stereo_calib_.left_camera_info_,
//                         right_image,
//                         stereo_calib_.right_camera_info_,
//                         stereo_matching_params),
//             imu_meas.timestamps_,
//             imu_meas.measurements_,
//             reinit_packet_));

//         last_timestamp_ = timestamp;
//         frame_count_++;

//       } else if (imu_query == utils::ThreadsafeImuBuffer::QueryResult::
//                                   kTooFewMeasurementsAvailable) {
//         ROS_WARN(
//             "Too few IMU measurements between next frame and "
//             "last frame. Skip frame.");
//         // remove next frame (this would usually for the first
//         // frame)
//         stereo_buffer_.removeNext();
//       } else if (imu_query == utils::ThreadsafeImuBuffer::QueryResult::
//                                   kDataNotYetAvailable) {
//         // kNotYetAvailable then just wait for next loop...
//         ROS_WARN_THROTTLE(100, "IMU data not yet available. Skip frame.");
//       }
//     }

//     // Publish VIO output if any.
//     FrontendOutput::Ptr frontend_output;
//     BackendOutput::Ptr backend_output;
//     MesherOutput::Ptr mesher_output;
//     if (getVioOutput(frontend_output, backend_output, mesher_output)) {
//       publishVioOutput(frontend_output, backend_output, mesher_output);
//     }

//     // Publish LCD output if any.
//     LcdOutput::Ptr lcd_output = nullptr;
//     if (lcd_output_queue_.pop(lcd_output)) {
//       publishLcdOutput(lcd_output);
//     }

//     // spin loop
//     ros::spinOnce();
//   }

//   ROS_INFO("Ros data source spin done. Shutting down vio output queue.");
//   backend_output_queue_.shutdown();
//   frontend_output_queue_.shutdown();
//   mesher_output_queue_.shutdown();
//   lcd_output_queue_.shutdown();

//   return true;
// }

}  // namespace VIO
