/**
 * @file   RosbagDataSource.cpp
 * @brief  Parse rosbad and run spark vio
 * @author Yun Chang
 */

#include "RosbagDataSource.h"

namespace VIO {

RosbagDataProvider::RosbagDataProvider(
    std::string left_camera_topic,
    std::string right_camera_topic,
    std::string imu_topic,
    std::string bag_input_path) :
    RosBaseDataProvider(left_camera_topic, right_camera_topic, imu_topic),
    rosbag_data_() {

  ROS_INFO("Starting SparkVIO wrapper for offline");

  RosBaseDataProvider::parseImuData(&rosbag_data_, &imu_params_);

  // parse data from rosbag
  parseRosbag(bag_input_path, left_camera_topic, right_camera_topic, imu_topic,
              &rosbag_data_);

  ROS_INFO(">>>>>>> Parsed rosbag data");

  // print parameters for check
  print();
}

RosbagDataProvider::~RosbagDataProvider() {}

bool RosbagDataProvider::parseRosbag(std::string bag_path,
                                     std::string left_imgs_topic,
                                     std::string right_imgs_topic,
                                     std::string imu_topic,
                                     RosbagData* rosbag_data) {
  // Fill in rosbag to data_
  rosbag::Bag bag;
  bag.open(bag_path);

  std::vector<std::string> topics;
  topics.push_back(left_imgs_topic);
  topics.push_back(right_imgs_topic);
  topics.push_back(imu_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool start_parsing_stereo = false; // Keep track of this since cannot process image before imu data
  Timestamp last_imu_timestamp = 0; // For some dataset, have duplicated measurements for same time
  for (rosbag::MessageInstance msg: view) {
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
      Timestamp imu_data_timestamp = imu_data->header.stamp.toNSec();
      if (imu_data_timestamp > last_imu_timestamp) {
        rosbag_data->imu_data_.imu_buffer_.addMeasurement(imu_data_timestamp,
                                                  imu_accgyr);
        last_imu_timestamp = imu_data_timestamp;
      }
      start_parsing_stereo = true;
    }

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
        ROS_WARN("Skipping frame since IMU data not yet available");
      }
    }
  }

  // Sanity check:
  ROS_ERROR_COND(rosbag_data->left_imgs_.size() == 0 ||
                 rosbag_data->right_imgs_.size() == 0,
                 "No images parsed from rosbag!");
  ROS_ERROR_COND(rosbag_data->imu_data_.imu_buffer_.size() <=
                 rosbag_data->left_imgs_.size(),
                 "Less than or equal number fo imu data as image data.");
}

bool RosbagDataProvider::spin() {
  Timestamp timestamp_last_frame = rosbag_data_.timestamps_.at(0);

  // Stereo matching parameters
  const StereoMatchingParams& stereo_matching_params =
      frontend_params_.getStereoMatchingParams();

  for (size_t k = 0; k < rosbag_data_.getNumberOfImages(); k++) {
    if (ros::ok()) {
      // Main spin of the data provider: Interpolates IMU data
      // and builds StereoImuSyncPacket
      // (Think of this as the spin of the other parser/data-providers)
      Timestamp timestamp_frame_k = rosbag_data_.timestamps_.at(k);
      // LOG(INFO) << k << " with timestamp: " << timestamp_frame_k;

      if (timestamp_frame_k > timestamp_last_frame) {
        ImuMeasurements imu_meas;
        utils::ThreadsafeImuBuffer::QueryResult imu_query =
            rosbag_data_.imu_data_.imu_buffer_.getImuDataInterpolatedUpperBorder(
              timestamp_last_frame,
              timestamp_frame_k,
              &imu_meas.timestamps_,
              &imu_meas.measurements_);
        if (imu_query == utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {

          // Call VIO Pipeline.
          VLOG(10) << "Call VIO processing for frame k: " << k
                   << " with timestamp: " << timestamp_frame_k << '\n'
                   << "////////////////////////////////// Creating packet!\n"
                   << "STAMPS IMU rows : \n" << imu_meas.timestamps_.rows() << '\n'
                   << "STAMPS IMU cols : \n" << imu_meas.timestamps_.cols() << '\n'
                   << "STAMPS IMU: \n" << imu_meas.timestamps_ << '\n'
                   << "ACCGYR IMU rows : \n" << imu_meas.measurements_.rows() << '\n'
                   << "ACCGYR IMU cols : \n" << imu_meas.measurements_.cols() << '\n'
                   << "ACCGYR IMU: \n" << imu_meas.measurements_ << '\n';

          timestamp_last_frame = timestamp_frame_k;

          vio_output_ = vio_callback_(StereoImuSyncPacket(
                            StereoFrame(k, timestamp_frame_k,
                            readRosImage(rosbag_data_.left_imgs_.at(k)),
                            stereo_calib_.left_camera_info_,
                            readRosImage(rosbag_data_.right_imgs_.at(k)),
                            stereo_calib_.right_camera_info_,
                            stereo_calib_.camL_Pose_camR_,
                            stereo_matching_params),
                            imu_meas.timestamps_,
                            imu_meas.measurements_));

          // Publish Output
          publishOutput();

          VLOG(10) << "Finished VIO processing for frame k = " << k;
        } else {
          ROS_WARN("Skipping frame %d. No imu data available between current frame and last frame", static_cast<int>(k));
        }
      } else {
        ROS_WARN("Skipping frame %d. Frame timestamp less than or equal to last frame.", static_cast<int>(k));
      }
    } else {
      break; 
    }
  }

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

  std::cout << "number of stereo frames: " << rosbag_data_.getNumberOfImages() << std::endl;
  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
}

} // End of VIO namespace
