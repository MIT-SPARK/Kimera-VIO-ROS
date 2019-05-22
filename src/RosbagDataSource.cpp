/**
 * @file   RosbagDataSource.cpp
 * @brief  Parse rosbad and run spark vio
 * @author Yun Chang
 */

#include "RosbagDataSource.h"

namespace VIO {

RosbagDataProvider::RosbagDataProvider(const std::string &left_camera_topic,
                                       const std::string &right_camera_topic,
                                       const std::string &imu_topic,
                                       const std::string &bag_input_path)
  : stereo_calib_(),
    DataProvider(),
    rosbag_data_() {
  ROS_INFO(">>>>>>> Initializing Spark-VIO <<<<<<<");
  // Parse calibration info for camera and IMU
  // Calibration info on parameter server (Parsed from yaml)
  parseCameraData(&stereo_calib_);
  parseImuData(&rosbag_data_, &imu_params_);

  ROS_INFO(">>>>>>> Parsed stereo and IMU parameters");
  // parse data from rosbag
  parseRosbag(bag_input_path, left_camera_topic, right_camera_topic, imu_topic,
              &rosbag_data_);

  ROS_INFO(">>>>>>> Parsed rosbag data");
  // print parameters for check
  print();
}

RosbagDataProvider::~RosbagDataProvider() {}

cv::Mat RosbagDataProvider::readRosImage(
    const sensor_msgs::ImageConstPtr& img_msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch(cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  return cv_ptr->image; // Return cv::Mat
}

bool RosbagDataProvider::parseCameraData(StereoCalibration* stereo_calib) {
  // Parse camera calibration info (from param server)

  // Rate
  double rate = 0.0;
  CHECK(nh_.getParam("camera_rate_hz", rate));

  // Resolution
  std::vector<int> resolution;
  CHECK(nh_.getParam("camera_resolution", resolution));

  // Get distortion/intrinsics/extrinsics for each camera
  for (int i = 0; i < 2; i++) {
    std::string camera_name = "";
    CameraParams camera_param_i;
    // Fill in rate and resolution
    camera_param_i.image_size_ = cv::Size(resolution.at(0), resolution.at(1));
    // Terminology wrong but following rest of the repo
    camera_param_i.frame_rate_ = 1.0 / rate;

    if (i == 0) {
      camera_name = "left_camera_";
    } else {
      camera_name = "right_camera_";
    }
    // Parse intrinsics (camera matrix)
    std::vector<double> intrinsics;
    CHECK(nh_.getParam(camera_name + "intrinsics", intrinsics));
    camera_param_i.intrinsics_ = intrinsics;
    // Convert intrinsics to camera matrix (OpenCV format)
    camera_param_i.camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_param_i.camera_matrix_.at<double>(0, 0) = intrinsics.at(0);
    camera_param_i.camera_matrix_.at<double>(1, 1) = intrinsics.at(1);
    camera_param_i.camera_matrix_.at<double>(0, 2) = intrinsics.at(2);
    camera_param_i.camera_matrix_.at<double>(1, 2) = intrinsics.at(3);

    // Parse extrinsics (rotation and translation)
    std::vector<double> extrinsics;
    std::vector<double> frame_change; // encode calibration frame to body frame
    CHECK(nh_.getParam(camera_name + "extrinsics", extrinsics));
    CHECK(nh_.getParam("calibration_to_body_frame", frame_change));
    // Place into matrix
    // 4 4 is hardcoded here because currently only accept extrinsic input
    // in homoegeneous format [R T ; 0 1]
    cv::Mat E_calib = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat calib2body = cv::Mat::zeros(4, 4, CV_64F);
    for (int k = 0; k < 16; k++) {
      int row = k / 4;
      int col = k % 4;
      E_calib.at<double>(row, col) = extrinsics.at(k);
      calib2body.at<double>(row, col) = frame_change.at(k);
    }
    cv::Mat E_body = calib2body * E_calib; // Extrinsics in body frame
    // restore back to vector form
    std::vector<double> extrinsics_body;
    for (int k = 0; k < 16; k++) {
      int row = k / 4;
      int col = k % 4;
      extrinsics_body.push_back(E_body.at<double>(row, col));
    }

    camera_param_i.body_Pose_cam_ = UtilsOpenCV::Vec2pose(extrinsics_body, 4, 4);

    // Parse distortion
    std::vector<double> d_coeff;
    CHECK(nh_.getParam(camera_name + "distortion_coefficients", d_coeff));
    cv::Mat distortion_coeff = cv::Mat::zeros(1, 5, CV_64F);

    ROS_INFO("DCOEFF SIZE: %d", int(d_coeff.size()));

    switch (d_coeff.size()) {
      case(1): // for FOV coefficient
        distortion_coeff.at<double>(0, 0) = d_coeff.at(0); // w
        ROS_INFO("Using FOV distortion model");
        break;

      case(4): // if given 4 coefficients
        distortion_coeff.at<double>(0, 0) = d_coeff.at(0); // k1
        distortion_coeff.at<double>(0, 1) = d_coeff.at(1); // k2
        distortion_coeff.at<double>(0, 3) = d_coeff.at(2); // p1
        distortion_coeff.at<double>(0, 4) = d_coeff.at(3); // p2
        ROS_INFO("Using radtan distortion model");
        break;

      case(5): // if given 5 coefficients
        for (int k = 0; k < 5; k++) {
          distortion_coeff.at<double>(0, k) = d_coeff.at(k); // k1, k2, k3, p1, p2
        }
        ROS_INFO("Using radtan distortion model");
        break;

      default: // otherwise
        ROS_FATAL("Unsupported distortion format.. why?");
    }

    camera_param_i.distortion_coeff_ = distortion_coeff;

    // TODO add skew (can add switch statement when parsing intrinsics)
    camera_param_i.calibration_ = gtsam::Cal3DS2(
        intrinsics[0], // fx
        intrinsics[1], // fy
        0.0,           // skew
        intrinsics[2], // u0
        intrinsics[3], // v0
        distortion_coeff.at<double>(0, 0),  //  k1
        distortion_coeff.at<double>(0, 1),  //  k2
        distortion_coeff.at<double>(0, 3),  //  p1
        distortion_coeff.at<double>(0, 4)); //  p2

    if (i == 0){
      stereo_calib->left_camera_info_ = camera_param_i;
    } else {
      stereo_calib->right_camera_info_ = camera_param_i;
    }

  }

  // Calculate the pose of right camera relative to the left camera
  stereo_calib->camL_Pose_camR_ = (stereo_calib->left_camera_info_.body_Pose_cam_).between(
                                    stereo_calib->right_camera_info_.body_Pose_cam_);

  ROS_INFO("Parsed stereo camera calibration");
  return true;
}

bool RosbagDataProvider::parseImuData(RosbagData* rosbag_data,
                                      ImuParams* imuparams) {
  CHECK_NOTNULL(rosbag_data);
  CHECK_NOTNULL(imuparams);
  // Parse IMU calibration info (from param server)
  double rate, rate_std, rate_maxMismatch, gyro_noise, gyro_walk, acc_noise, acc_walk;

  std::vector<double> extrinsics;

  CHECK(nh_.getParam("imu_rate_hz", rate));
  CHECK(nh_.getParam("gyroscope_noise_density", gyro_noise));
  CHECK(nh_.getParam("gyroscope_random_walk", gyro_walk));
  CHECK(nh_.getParam("accelerometer_noise_density", acc_noise));
  CHECK(nh_.getParam("accelerometer_random_walk", acc_walk));

  // TODO: We should probably remove this! This is not parsed in anyway to the pipeline!!
  CHECK(nh_.getParam("imu_extrinsics", extrinsics));

  // TODO: Do we need these parameters??
  rosbag_data->imu_data_.nominal_imu_rate_ = 1.0 / rate;
  rosbag_data->imu_data_.imu_rate_ = 1.0 / rate;
  rosbag_data->imu_data_.imu_rate_std_ = 0.00500009; // set to 0 for now
  rosbag_data->imu_data_.imu_rate_maxMismatch_ = 0.00500019; // set to 0 for now

  // Gyroscope and accelerometer noise parameters
  imuparams->gyro_noise_ = gyro_noise;
  imuparams->gyro_walk_ = gyro_walk;
  imuparams->acc_noise_ = acc_noise;
  imuparams->acc_walk_ = acc_walk;

  // Value parsed from vioBackend
  std::string vio_params_path;
  CHECK(nh_.getParam("vio_params_filepath", vio_params_path));
  VioBackEndParams backend_params;
  backend_params.parseYAML(vio_params_path);
  imuparams->imu_integration_sigma_ = backend_params.imuIntegrationSigma_;
  imuparams->n_gravity_ = backend_params.n_gravity_;

  // Expects imu frame to be aligned with body frame

  ROS_INFO("Parsed IMU calibration");
  return true;
}

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

        vio_callback_(StereoImuSyncPacket(
                        StereoFrame(k, timestamp_frame_k,
                                    readRosImage(rosbag_data_.left_imgs_.at(k)),
                                    stereo_calib_.left_camera_info_,
                                    readRosImage(rosbag_data_.right_imgs_.at(k)),
                                    stereo_calib_.right_camera_info_,
                                    stereo_calib_.camL_Pose_camR_,
                                    stereo_matching_params),
                        imu_meas.timestamps_,
                        imu_meas.measurements_));

        VLOG(10) << "Finished VIO processing for frame k = " << k;
      } else {
        ROS_WARN("Skipping frame %d. No imu data available between current frame and last frame", static_cast<int>(k));
      }
    } else {
      ROS_WARN("Skipping frame %d. Frame timestamp less than or equal to last frame.", static_cast<int>(k));
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
