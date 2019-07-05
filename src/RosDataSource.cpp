/**
 * @file   RosDataSource.cpp
 * @brief  ROS wrapper
 * @author Yun Chang
 */

#include "RosDataSource.h"
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>

namespace VIO {

RosDataProvider::RosDataProvider(
    std::string left_camera_topic, std::string right_camera_topic,
    std::string imu_topic,
    std::string reinit_flag_topic = "/sparkvio/reinit_flag",
    std::string reinit_pose_topic = "/sparkvio/reinit_pose")
    : RosBaseDataProvider(left_camera_topic, right_camera_topic, imu_topic),
      it_(nh_cam_),
      // Image subscriber (left)
      left_img_subscriber_(it_, left_camera_topic, 1),
      // Image subscriber (right)
      right_img_subscriber_(it_, right_camera_topic, 1),
      sync(sync_pol(10), left_img_subscriber_, right_img_subscriber_),
      // initialize last timestamp (img) to be 0
      last_time_stamp_(0),
      // initialize last timestamp (imu) to be 0
      last_imu_time_stamp_(0),
      
      // keep track of number of frames processed)
      frame_count_(1) {

  ROS_INFO("Starting SparkVIO wrapper for online");

  parseImuData(&imu_data_, &imu_params_);
  // print parameters for check
  print();

  // Start IMU subscriber

  imu_subscriber_ =
      nh_imu_.subscribe(imu_topic, 50, &RosDataProvider::callbackIMU, this);

  // Define Callback Queue for IMU Data
  ros::CallbackQueue imu_queue;
  nh_imu_.setCallbackQueue(&imu_queue);

  // Spawn Async Spinner (Running on Custom Queue) for IMU
  // 0 to use number of processor queues

  ros::AsyncSpinner async_spinner_imu(0, &imu_queue);
  async_spinner_imu.start();

  ////// Synchronize stero image callback

  sync.registerCallback(
      boost::bind(&RosDataProvider::callbackCamAndProcessStereo, this, _1, _2));

  // Define Callback Queue for Cam Data
  ros::CallbackQueue cam_queue;
  nh_cam_.setCallbackQueue(&cam_queue);

  // Spawn Async Spinner (Running on Custom Queue) for Cam
  ros::AsyncSpinner async_spinner_cam(0, &cam_queue);
  async_spinner_cam.start();

  ////// Define Reinitializer Subscriber
  reinit_flag_topic_ = reinit_flag_topic;
  reinit_pose_topic_ = reinit_pose_topic;

  // Start reinitializer subscriber
  reinit_flag_subscriber_ = nh_reinit_.subscribe(reinit_flag_topic, 10,
                                      &RosDataProvider::callbackReinit, this);

  // Start reinitializer pose subscriber
  reinit_pose_subscriber_ = nh_reinit_.subscribe(
      reinit_pose_topic, 10, &RosDataProvider::callbackReinitPose, this);

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
  ros::Duration imu_shift(imu_params_.imu_shift_);
  Timestamp timestamp = msgIMU->header.stamp.toNSec() - imu_shift.toNSec();
  // t_imu = t_cam + imu_shift (see: Kalibr)
  // ROS_INFO("Recieved message at time %ld", timestamp);

  // add measurement to buffer
  // time strictly increasing
  if (timestamp > last_imu_time_stamp_) {
    imu_data_.imu_buffer_.addMeasurement(timestamp, imu_accgyr);
  }

  if (last_time_stamp_ == 0) { // initialize first img time stamp
    last_time_stamp_ = timestamp;
  }

  last_imu_time_stamp_ = timestamp;
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

  gtsam::Point3 position(reinitPose.pose.position.x,
                          reinitPose.pose.position.y,
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
    const Timestamp timestamp = stereo_buffer_.getEarliestTimestamp();
    if (timestamp <= last_time_stamp_) {
      if (stereo_buffer_.size() != 0) {
        ROS_WARN("Next frame in image buffer is from the same or "
                 "earlier time than the last processed frame. Skip "
                 "frame.");
        // remove next frame (this would usually for the first
        // frame)
        stereo_buffer_.removeNext();
      }
      // else just waiting for next stereo frames
    } else {
      // Test if IMU data available
      ImuMeasurements imu_meas;

      utils::ThreadsafeImuBuffer::QueryResult imu_query =
          imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
              last_time_stamp_, timestamp, &imu_meas.timestamps_,
              &imu_meas.measurements_);
      if (imu_query ==
          utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
        // data available
        sensor_msgs::ImageConstPtr left_ros_img, right_ros_img;
        stereo_buffer_.extractLatestImages(left_ros_img, right_ros_img);

        const StereoMatchingParams &stereo_matching_params =
            frontend_params_.getStereoMatchingParams();

        // Read ROS images to cv type
        cv::Mat left_image, right_image;

        switch (stereo_matching_params.vision_sensor_type_) {
        case VisionSensorType::STEREO:
          // no conversion
          left_image = readRosImage(left_ros_img);
          right_image = readRosImage(right_ros_img);
          break;
        case VisionSensorType::RGBD: // just use depth to "fake
                                     // right pixel matches" apply
                                     // conversion
          left_image = readRosRGBImage(left_ros_img);
          right_image = readRosDepthImage(right_ros_img);
          break;
        default:
          LOG(FATAL) << "vision sensor type not recognised.";
          break;
        }
        vio_output_ = vio_callback_(StereoImuSyncPacket(
            StereoFrame(frame_count_, timestamp, left_image,
                        stereo_calib_.left_camera_info_, right_image,
                        stereo_calib_.right_camera_info_,
                        stereo_calib_.camL_Pose_camR_, stereo_matching_params),
            imu_meas.timestamps_, imu_meas.measurements_, reinit_packet_));
        // Reset reinit flag for reinit packet
        resetReinitFlag();

        // Publish all outputs
        publishOutput();

        last_time_stamp_ = timestamp;
        frame_count_++;

      } else if (imu_query == utils::ThreadsafeImuBuffer::QueryResult::
                                  kTooFewMeasurementsAvailable) {
        ROS_WARN("Too few IMU measurements between next frame and "
                 "last frame. Skip frame.");
        // remove next frame (this would usually for the first
        // frame)
        stereo_buffer_.removeNext();
      }

      // else it would be the kNotYetAvailable then just wait for
      // next loop
    }

    // spin loop
    ros::spinOnce();
  }

  ROS_INFO("Done.");
  return true;
}

void RosDataProvider::print() const {
  std::cout << ">>>>>>>>> RosDataProvider::print <<<<<<<<<<<" << std::endl;
  stereo_calib_.camL_Pose_camR_.print("camL_Pose_calR \n");
  // For each of the 2 cameras.
  std::cout << ">> Left camera params <<" << std::endl;
  stereo_calib_.left_camera_info_.print();
  std::cout << ">> Right camera params <<" << std::endl;
  stereo_calib_.right_camera_info_.print();
  std::cout << ">> IMU info << " << std::endl;
  imu_data_.print();
  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
}

} // namespace VIO
