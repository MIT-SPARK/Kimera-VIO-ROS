/**
 * @file   rosbag-data-source.cpp
 * @brief  Parse rosbad and run spark vio
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#include "spark-vio-ros/rosbag-data-source.h"

#include <sensor_msgs/Imu.h>

namespace VIO {

RosbagDataProvider::RosbagDataProvider()
    : RosBaseDataProvider(), rosbag_data_() {
  ROS_INFO("Starting SparkVIO wrapper for offline");

  std::string rosbag_path;
  CHECK(nh_private_.getParam("rosbag_path", rosbag_path));
  std::string left_camera_topic;
  CHECK(nh_private_.getParam("left_cam_rosbag_topic", left_camera_topic));
  std::string right_camera_topic;
  CHECK(nh_private_.getParam("right_cam_rosbag_topic", right_camera_topic));
  std::string imu_topic;
  CHECK(nh_private_.getParam("imu_rosbag_topic", imu_topic));

  parseImuData(&rosbag_data_, &imu_params_);

  // Parse data from rosbag
  CHECK(parseRosbag(rosbag_path, left_camera_topic, right_camera_topic,
                    imu_topic, &rosbag_data_));

  ROS_INFO(">>>>>>> Parsed rosbag data");

  // Print parameters for check
  print();
}

RosbagDataProvider::~RosbagDataProvider() {}

bool RosbagDataProvider::parseRosbag(const std::string& bag_path,
                                     const std::string& left_imgs_topic,
                                     const std::string& right_imgs_topic,
                                     const std::string& imu_topic,
                                     RosbagData* rosbag_data) {
  // Fill in rosbag to data_
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(left_imgs_topic);
  topics.push_back(right_imgs_topic);
  topics.push_back(imu_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool start_parsing_stereo =
      false;  // Keep track of this since cannot process image before imu data
  Timestamp last_imu_timestamp =
      0;  // For some dataset, have duplicated measurements for same time
  for (const rosbag::MessageInstance& msg : view) {
    // Get topic.
    const std::string& msg_topic = msg.getTopic();

    // IMU
    sensor_msgs::ImuConstPtr imu_data = msg.instantiate<sensor_msgs::Imu>();
    if (imu_data != nullptr && msg_topic == imu_topic) {
      gtsam::Vector6 imu_accgyr;
      imu_accgyr(0) = imu_data->linear_acceleration.x;
      imu_accgyr(1) = imu_data->linear_acceleration.y;
      imu_accgyr(2) = imu_data->linear_acceleration.z;
      imu_accgyr(3) = imu_data->angular_velocity.x;
      imu_accgyr(4) = imu_data->angular_velocity.y;
      imu_accgyr(5) = imu_data->angular_velocity.z;
      const Timestamp& imu_data_timestamp = imu_data->header.stamp.toNSec();
      if (imu_data_timestamp > last_imu_timestamp) {
        rosbag_data->imu_data_.imu_buffer_.addMeasurement(imu_data_timestamp,
                                                          imu_accgyr);
        last_imu_timestamp = imu_data_timestamp;
      } else {
        ROS_FATAL(
            "IMU timestamps in rosbag are out of order: consider re-ordering "
            "rosbag.");
      }
      start_parsing_stereo = true;
    } else {
      // Check if msg is an image.
      sensor_msgs::ImageConstPtr img = msg.instantiate<sensor_msgs::Image>();
      if (img != nullptr) {
        if (start_parsing_stereo) {
          // Check left or right image.
          if (msg_topic == left_imgs_topic) {
            // Timestamp is in nanoseconds
            rosbag_data->timestamps_.push_back(img->header.stamp.toNSec());
            rosbag_data->left_imgs_.push_back(img);
          } else if (msg_topic == right_imgs_topic) {
            rosbag_data->right_imgs_.push_back(img);
          } else {
            ROS_WARN_STREAM("Img with unexpected topic: " << msg_topic);
          }
        } else {
          ROS_WARN(
              "Skipping first frame in rosbag, since IMU data not yet "
              "available.");
        }
      }
    }
  }
  bag.close();

  // Sanity check:
  ROS_ERROR_COND(rosbag_data->left_imgs_.size() == 0 ||
                     rosbag_data->right_imgs_.size() == 0,
                 "No images parsed from rosbag!");
  ROS_ERROR_COND(rosbag_data->imu_data_.imu_buffer_.size() <=
                     rosbag_data->left_imgs_.size(),
                 "Less than or equal number fo imu data as image data.");
  return true;
}

bool RosbagDataProvider::spin() {
  Timestamp timestamp_last_frame = rosbag_data_.timestamps_.at(0);

  // Stereo matching parameters
  const StereoMatchingParams& stereo_matching_params =
      frontend_params_.getStereoMatchingParams();

  for (size_t k = 0; k < rosbag_data_.getNumberOfImages(); k++) {
    if (nh_.ok() && ros::ok() && !ros::isShuttingDown()) {
      // Main spin of the data provider: Interpolates IMU data
      // and builds StereoImuSyncPacket
      // (Think of this as the spin of the other parser/data-providers)
      const Timestamp& timestamp_frame_k = rosbag_data_.timestamps_.at(k);
      if (timestamp_frame_k > timestamp_last_frame) {
        ImuMeasurements imu_meas;
        utils::ThreadsafeImuBuffer::QueryResult imu_query =
            rosbag_data_.imu_data_.imu_buffer_
                .getImuDataInterpolatedUpperBorder(
                    timestamp_last_frame, timestamp_frame_k,
                    &imu_meas.timestamps_, &imu_meas.measurements_);
        if (imu_query ==
            utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
          // Call VIO Pipeline.
          VLOG(10) << "Call VIO processing for frame k: " << k
                   << " with timestamp: " << timestamp_frame_k << '\n'
                   << "////////////////////////////////// Creating packet!\n"
                   << "STAMPS IMU rows : \n"
                   << imu_meas.timestamps_.rows() << '\n'
                   << "STAMPS IMU cols : \n"
                   << imu_meas.timestamps_.cols() << '\n'
                   << "STAMPS IMU: \n"
                   << imu_meas.timestamps_ << '\n'
                   << "ACCGYR IMU rows : \n"
                   << imu_meas.measurements_.rows() << '\n'
                   << "ACCGYR IMU cols : \n"
                   << imu_meas.measurements_.cols() << '\n'
                   << "ACCGYR IMU: \n"
                   << imu_meas.measurements_ << '\n';

          timestamp_last_frame = timestamp_frame_k;

          // Publish Output
          vio_callback_(StereoImuSyncPacket(
              StereoFrame(k, timestamp_frame_k,
                          readRosImage(rosbag_data_.left_imgs_.at(k)),
                          stereo_calib_.left_camera_info_,
                          readRosImage(rosbag_data_.right_imgs_.at(k)),
                          stereo_calib_.right_camera_info_,
                          stereo_calib_.camL_Pose_camR_,
                          stereo_matching_params),
              imu_meas.timestamps_, imu_meas.measurements_));
          VLOG(10) << "Finished VIO processing for frame k = " << k;
        } else {
          ROS_WARN(
              "Skipping frame %d. No imu data available between current frame "
              "and last frame",
              static_cast<int>(k));
        }
      } else {
        ROS_WARN(
            "Skipping frame %d. Frame timestamps out of order:"
            " less than or equal to last frame.",
            static_cast<int>(k));
      }

      // Publish VIO output if any.
      // TODO(Toni) this could go faster if running in another thread or node...
      SpinOutputPacket vio_output;
      if (vio_output_queue_.pop(vio_output)) publishOutput(vio_output);

      ros::spinOnce();
    } else {
      LOG(ERROR) << "ROS SHUTDOWN requested, stopping rosbag spin.";
      ros::shutdown();
      return false;
    }
  }
  LOG(WARNING) << "Rosbag processing finished.";
  return true;
}

bool RosbagDataProvider::parseImuData(RosbagData* rosbag_data,
                                      ImuParams* imuparams) {
  CHECK_NOTNULL(rosbag_data);
  CHECK_NOTNULL(imuparams);
  // Parse IMU calibration info (from param server)
  double rate, rate_std, rate_maxMismatch, gyro_noise, gyro_walk, acc_noise,
      acc_walk;

  std::vector<double> extrinsics;

  CHECK(nh_private_.getParam("imu_rate_hz", rate));
  CHECK(nh_private_.getParam("gyroscope_noise_density", gyro_noise));
  CHECK(nh_private_.getParam("gyroscope_random_walk", gyro_walk));
  CHECK(nh_private_.getParam("accelerometer_noise_density", acc_noise));
  CHECK(nh_private_.getParam("accelerometer_random_walk", acc_walk));

  // TODO: We should probably remove this! This is not parsed in anyway to the
  // pipeline!!
  CHECK(nh_private_.getParam("imu_extrinsics", extrinsics));

  // TODO: Do we need these parameters??
  rosbag_data->imu_data_.nominal_imu_rate_ = 1.0 / rate;
  rosbag_data->imu_data_.imu_rate_ = 1.0 / rate;
  rosbag_data->imu_data_.imu_rate_std_ = 0.00500009;  // set to 0 for now
  rosbag_data->imu_data_.imu_rate_maxMismatch_ =
      0.00500019;  // set to 0 for now

  // Gyroscope and accelerometer noise parameters
  imuparams->gyro_noise_ = gyro_noise;
  imuparams->gyro_walk_ = gyro_walk;
  imuparams->acc_noise_ = acc_noise;
  imuparams->acc_walk_ = acc_walk;

  ROS_INFO("Parsed IMU calibration");
  return true;
}

void RosbagDataProvider::print() const {
  std::cout << "\n " << std::endl;
  std::cout << ">>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
  std::cout << ">>>>>>>>> RosbagDataProvider::print <<<<<<<<<<<" << std::endl;
  stereo_calib_.camL_Pose_camR_.print("camL_Pose_calR \n");
  // For each of the 2 cameras.
  std::cout << ">> Left camera params <<" << std::endl;
  stereo_calib_.left_camera_info_.print();
  std::cout << ">> Right camera params <<" << std::endl;
  stereo_calib_.right_camera_info_.print();
  std::cout << ">> IMU info << " << std::endl;
  rosbag_data_.imu_data_.print();

  std::cout << "number of stereo frames: " << rosbag_data_.getNumberOfImages()
            << std::endl;
  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
}

}  // namespace VIO
