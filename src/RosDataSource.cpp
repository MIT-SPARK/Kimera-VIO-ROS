/**
 * @file   RosDataSource.cpp
 * @brief  ROS wrapper
 * @author Yun Chang
 */
#include "RosDataSource.h"
#include <string>
#include <vector>

namespace VIO {

RosDataProvider::RosDataProvider(
    std::string left_camera_topic, std::string right_camera_topic,
    std::string imu_topic,
    std::string reinit_flag_topic = "/sparkvio/reinit_flag",
    std::string reinit_pose_topic = "/sparkvio/reinit_pose")
    : stereo_calib_(), DataProvider(), it_(nh_cam_),
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
      frame_count_(0),
      // default constructor
      vio_output_() {
  ROS_INFO(">>>>>>> Initializing Spark-VIO <<<<<<<");
  // Parse calibration info for camera and IMU
  // Calibration info on parameter server (Parsed from yaml)
  parseCameraData(&stereo_calib_);
  parseImuData(&imuData_, &imuParams_);

  // print parameters for check
  print();

  ////// Define IMU Subscriber

  imu_topic_ = imu_topic;

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

  // Start odometry publisher
  CHECK(nh_.getParam("odom_base_frame_id", odom_base_frame_id_));
  CHECK(nh_.getParam("odom_child_frame_id", odom_child_frame_id_));
  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(
            "sparkvio/odometry", 10);

  // Start frontend stats publisher
  frontend_stats_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "sparkvio/frontend_stats", 10);

  // Start resiliency publisher
  resil_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "sparkvio/resiliency", 10);

  // Start imu bias publisher
  bias_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "sparkvio/imu_bias", 10);

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

cv::Mat
RosDataProvider::readRosImage(const sensor_msgs::ImageConstPtr &img_msg) {
  // Use cv_bridge to read ros image to cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch(cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  // Return cv::Mat
  return cv_ptr->image;
}

cv::Mat
RosDataProvider::readRosRGBImage(const sensor_msgs::ImageConstPtr &img_msg) {
  // Use cv_bridge to read ros image to cv::Mat
  cv::Mat img_rgb = RosDataProvider::readRosImage(img_msg);
  // CV_RGB2GRAY);
  cv::cvtColor(img_rgb, img_rgb, cv::COLOR_BGR2GRAY);
  // Return cv::Mat
  return img_rgb;
}

cv::Mat
RosDataProvider::readRosDepthImage(const sensor_msgs::ImageConstPtr &img_msg) {
  // Use cv_bridge to read ros image to cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr =
        cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch(cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  cv::Mat img_depth = cv_ptr->image;
  if (img_depth.type() != CV_16UC1)
    // mDepthMapFactor);
    img_depth.convertTo(img_depth, CV_16UC1);
  // Return cv::Mat
  return img_depth;
}

bool RosDataProvider::parseCameraData(StereoCalibration* stereo_calib) {
  // Parse camera calibration info (from param server)

  // Rate
  double rate;
  nh_.getParam("camera_rate_hz", rate);

  // Resoltuion
  std::vector<int> resolution;
  nh_.getParam("camera_resolution", resolution);

  // Get distortion/intrinsics/extrinsics for each camera
  for (int i = 0; i < 2; i++) {
    std::string camera_name;
    CameraParams camera_param_i;
    // Fill in rate and resolution
    camera_param_i.image_size_ = cv::Size(resolution[0], resolution[1]);
    // Terminology wrong but following rest of the repo
    camera_param_i.frame_rate_ = 1.0 / rate;

    if (i == 0) {
      camera_name = "left_camera_";
    } else {
      camera_name = "right_camera_";
    }
    // Parse intrinsics (camera matrix)
    std::vector<double> intrinsics;
    nh_.getParam(camera_name + "intrinsics", intrinsics);
    camera_param_i.intrinsics_ = intrinsics;
    // Conver intrinsics to camera matrix (OpenCV format)
    camera_param_i.camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_param_i.camera_matrix_.at<double>(0, 0) = intrinsics[0];
    camera_param_i.camera_matrix_.at<double>(1, 1) = intrinsics[1];
    camera_param_i.camera_matrix_.at<double>(0, 2) = intrinsics[2];
    camera_param_i.camera_matrix_.at<double>(1, 2) = intrinsics[3];

    // Parse extrinsics (rotation and translation)
    std::vector<double> extrinsics;
    // Encode calibration frame to body frame
    std::vector<double> frame_change;
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
      E_calib.at<double>(row, col) = extrinsics[k];
      calib2body.at<double>(row, col) = frame_change[k];
    }

    // TODO(Yun): Check frames convention!
    // Extrinsics in body frame
    cv::Mat E_body = calib2body * E_calib;

    // restore back to vector form
    std::vector<double> extrinsics_body;
    for (int k = 0; k < 16; k++) {
      int row = k / 4;
      int col = k % 4;
      extrinsics_body.push_back(E_body.at<double>(row, col));
    }

    camera_param_i.body_Pose_cam_ =
        UtilsOpenCV::Vec2pose(extrinsics_body, 4, 4);

    // Distortion model
    std::string distortion_model;
    nh_.getParam("distortion_model", distortion_model);
    camera_param_i.distortion_model_ = distortion_model;

    // Parse distortion
    std::vector<double> d_coeff;
    nh_.getParam(camera_name + "distortion_coefficients", d_coeff);
    cv::Mat distortion_coeff;

    switch (d_coeff.size()) {
    // if given 4 coefficients
    case (4):
      ROS_INFO(
          "using radtan or equidistant model (4 coefficients) for camera %d",
          i);
      distortion_coeff = cv::Mat::zeros(1, 4, CV_64F);
      distortion_coeff.at<double>(0, 0) = d_coeff[0]; // k1
      distortion_coeff.at<double>(0, 1) = d_coeff[1]; // k2
      distortion_coeff.at<double>(0, 3) = d_coeff[2]; // p1 or k3
      distortion_coeff.at<double>(0, 4) = d_coeff[3]; // p2 or k4
      break;

    case (5): // if given 5 coefficients
      ROS_INFO("using radtan model (5 coefficients) for camera %d", i);
      distortion_coeff = cv::Mat::zeros(1, 5, CV_64F);
      for (int k = 0; k < 5; k++) {
        distortion_coeff.at<double>(0, k) = d_coeff[k]; // k1, k2, k3, p1, p2
      }
      break;

    default: // otherwise
      ROS_FATAL("Unsupported distortion format");
    }

    camera_param_i.distortion_coeff_ = distortion_coeff;

    // TODO(unknown): add skew (can add switch statement when parsing
    // intrinsics)
    camera_param_i.calibration_ =
        gtsam::Cal3DS2(intrinsics[0],                      // fx
                       intrinsics[1],                      // fy
                       0.0,                                // skew
                       intrinsics[2],                      // u0
                       intrinsics[3],                      // v0
                       distortion_coeff.at<double>(0, 0),  //  k1
                       distortion_coeff.at<double>(0, 1),  //  k2
                       distortion_coeff.at<double>(0, 3),  //  p1
                       distortion_coeff.at<double>(0, 4)); //  p2

    if (i == 0) {
      stereo_calib->left_camera_info_ = camera_param_i;
    } else {
      stereo_calib->right_camera_info_ = camera_param_i;
    }
  }

  // Calculate the pose of right camera relative to the left camera
  stereo_calib->camL_Pose_camR_ =
      (stereo_calib->left_camera_info_.body_Pose_cam_)
          .between(stereo_calib->right_camera_info_.body_Pose_cam_);

  ROS_INFO("Parsed stereo camera calibration");
  return true;
}

bool RosDataProvider::parseImuData(ImuData* imudata, ImuParams* imuparams) {
  // Parse IMU calibration info (from param server)
  double rate, rate_std, rate_maxMismatch, gyro_noise, gyro_walk, acc_noise,
      acc_walk, imu_shift;

  std::vector<double> extrinsics;

  CHECK(nh_.getParam("imu_rate_hz", rate));
  CHECK(nh_.getParam("gyroscope_noise_density", gyro_noise));
  CHECK(nh_.getParam("gyroscope_random_walk", gyro_walk));
  CHECK(nh_.getParam("accelerometer_noise_density", acc_noise));
  CHECK(nh_.getParam("accelerometer_random_walk", acc_walk));
  CHECK(nh_.getParam("imu_extrinsics", extrinsics));
  CHECK(nh_.getParam("imu_shift", imu_shift));

  // TODO(Sandro): Do we need these parameters??
  imudata->nominal_imu_rate_ = 1.0 / rate;
  imudata->imu_rate_ = 1.0 / rate;
  imudata->imu_rate_std_ = 0.00500009; // set to 0 for now
  imudata->imu_rate_maxMismatch_ = 0.00500019; // set to 0 for now

  // Gyroscope and accelerometer noise parameters
  imuparams->gyro_noise_ = gyro_noise;
  imuparams->gyro_walk_ = gyro_walk;
  imuparams->acc_noise_ = acc_noise;
  imuparams->acc_walk_ = acc_walk;
  imuparams->imu_shift_ =
      imu_shift; // Defined as t_imu = t_cam + imu_shift (see: Kalibr)

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
  ros::Duration imu_shift(imuParams_.imu_shift_);
  Timestamp timestamp = msgIMU->header.stamp.toNSec() - imu_shift.toNSec();
  // t_imu = t_cam + imu_shift (see: Kalibr)
  // ROS_INFO("Recieved message at time %ld", timestamp);

  // add measurement to buffer
  // time strictly increasing
  if (timestamp > last_imu_time_stamp_) {
    imuData_.imu_buffer_.addMeasurement(timestamp, imu_accgyr);
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
          imuData_.imu_buffer_.getImuDataInterpolatedUpperBorder(
              last_time_stamp_, timestamp, &imu_meas.timestamps_,
              &imu_meas.measurements_);
      if (imu_query ==
          utils::ThreadsafeImuBuffer::QueryResult::kDataAvailable) {
        // data available
        sensor_msgs::ImageConstPtr left_ros_img, right_ros_img;
        stereo_buffer_.extractLatestImages(left_ros_img, right_ros_img);

        // TODO(Yun): Move this to a different place, not in the loop?
        // Stereo matching parameters
        std::string tracker_params_path;
        CHECK(nh_.getParam("tracker_params_filepath", tracker_params_path));
        VioFrontEndParams frontend_params;
        frontend_params.parseYAML(tracker_params_path);
        const StereoMatchingParams &stereo_matching_params =
            frontend_params.getStereoMatchingParams();

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
          break;
        default:
          LOG(FATAL) << "vision sensor type not recognised.";
          break;
        }

        /* std::string img_name_left =
            "/home/sb/Desktop/debugging/pinhole_rgbd/"
            "left.png"; // TODO(Yun): Remove after testing
        cv::imwrite(img_name_left, left_image);
        std::string img_name_right =
            "/home/sb/Desktop/debugging/pinhole_rgbd/"
            "right.png"; // TODO: Remove after testing
        cv::imwrite(img_name_right, right_image); */

        // TODO(Sandro): Modify if we don't want to use the trivial reinit
        // pose
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

/*void RosDataProvider::publishOutput(const SpinOutputContainer& vio_output_) const {
  
  gtsam::Pose pose = vio_output_.W_Pose_Blkf_;
  gtsam::Vector3 velocity = vio_output_.W_Vel_Blkf_;
  Timestamp ts = vio_output_.timestamp_kf_;
*/

void RosDataProvider::publishOutput() {
  // Publish Output
  publishState();

  // Publish Frontend Stats
  publishFrontendStats();

  // Publish Resiliency
  publishResiliency();

  // Publish imu bias
  publishImuBias();
}

void RosDataProvider::publishState() {
  // Get latest estimates for odometry
  gtsam::Pose3 pose = vio_output_.getEstimatedPose();
  gtsam::Vector3 velocity = vio_output_.getEstimatedVelocity();
  Timestamp ts = vio_output_.getTimestamp();
  gtsam::Matrix6 pose_cov = vio_output_.getEstimatedPoseCov();
  gtsam::Matrix3 vel_cov = vio_output_.getEstimatedVelCov();

  // First publish odometry estimate
  nav_msgs::Odometry odometry_msg;

  long int sec = ts / 1e9;
  long int nsec = ts - sec * 1e9;

  // create header
  odometry_msg.header.stamp.sec = sec;
  odometry_msg.header.stamp.nsec = nsec;
  odometry_msg.header.frame_id = odom_base_frame_id_;
  odometry_msg.child_frame_id = odom_child_frame_id_;

  // position
  odometry_msg.pose.pose.position.x = pose.x();
  odometry_msg.pose.pose.position.y = pose.y();
  odometry_msg.pose.pose.position.z = pose.z();

  // orientation
  odometry_msg.pose.pose.orientation.w = pose.rotation().toQuaternion().w();
  odometry_msg.pose.pose.orientation.x = pose.rotation().toQuaternion().x();
  odometry_msg.pose.pose.orientation.y = pose.rotation().toQuaternion().y();
  odometry_msg.pose.pose.orientation.z = pose.rotation().toQuaternion().z();

  // Remap covariance from GTSAM convention
  // to odometry convention and fill in covariance
  std::vector<int> remapping{3, 4, 5, 0, 1, 2};
  // Position covariance first, angular covariance after
  CHECK_EQ(pose_cov.rows(), remapping.size());
  CHECK_EQ(pose_cov.rows() * pose_cov.cols(),
           odometry_msg.pose.covariance.size());
  for (int i = 0; i < pose_cov.rows(); i++) {
    for (int j = 0; j < pose_cov.cols(); j++) {
      odometry_msg.pose
          .covariance[remapping[i] * pose_cov.cols() + remapping[j]] =
          pose_cov(i, j);
    }
  }

  // Linear velocities, trivial values for angular
  Vector3 velocity_body = pose.rotation().transpose()*velocity;
  odometry_msg.twist.twist.linear.x = velocity_body(0);
  odometry_msg.twist.twist.linear.y = velocity_body(1);
  odometry_msg.twist.twist.linear.z = velocity_body(2);

  // Velocity covariance: first linear
  // and then angular (trivial values for angular)
  gtsam::Matrix3 vel_cov_body =
      pose.rotation().transpose().matrix() * vel_cov * pose.rotation().matrix();
  CHECK_EQ(vel_cov_body.rows(), 3);
  CHECK_EQ(vel_cov_body.cols(), 3);
  CHECK_EQ(odometry_msg.twist.covariance.size(), 36);
  for (int i = 0; i < vel_cov_body.rows(); i++) {
    for (int j = 0; j < vel_cov_body.cols(); j++) {
      odometry_msg.twist
          .covariance[i * static_cast<int>(
                              sqrt(odometry_msg.twist.covariance.size())) +
                      j] = vel_cov_body(i, j);
    }
  }
  // Publish message
  odom_publisher_.publish(odometry_msg);
}

void RosDataProvider::publishFrontendStats() {
  // Get frontend data for resiliency output
  DebugTrackerInfo debug_tracker_info = vio_output_.getTrackerInfo();

  // Create message type
  std_msgs::Float64MultiArray frontend_stats_msg;

  // Build Message Layout
  frontend_stats_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  frontend_stats_msg.layout.dim[0].size = 13;
  frontend_stats_msg.layout.dim[0].stride = 1;
  frontend_stats_msg.layout.dim[0].label =
      "FrontEnd: nrDetFeat, nrTrackFeat, nrMoIn, nrMoPu, nrStIn, nrStPu, "
      "moRaIt, stRaIt, nrVaRKP, nrNoLRKP, nrNoRRKP, nrNoDRKP nrFaARKP";

  // Get FrontEnd Statistics to Publish
  frontend_stats_msg.data.push_back(debug_tracker_info.nrDetectedFeatures_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrTrackerFeatures_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrMonoInliers_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrMonoPutatives_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrStereoInliers_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrStereoPutatives_);
  frontend_stats_msg.data.push_back(debug_tracker_info.monoRansacIters_);
  frontend_stats_msg.data.push_back(debug_tracker_info.stereoRansacIters_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrValidRKP_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrNoLeftRectRKP_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrNoRightRectRKP_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrNoDepthRKP_);
  frontend_stats_msg.data.push_back(debug_tracker_info.nrFailedArunRKP_);

  // Publish Message
  frontend_stats_publisher_.publish(frontend_stats_msg);
}

void RosDataProvider::publishResiliency() {
  // Get frontend and velocity covariance data for resiliency output
  DebugTrackerInfo debug_tracker_info = vio_output_.getTrackerInfo();
  gtsam::Matrix3 vel_cov = vio_output_.getEstimatedVelCov();
  gtsam::Matrix6 pose_cov = vio_output_.getEstimatedPoseCov();

  // Create message type for quality of SparkVIO
  std_msgs::Float64MultiArray resiliency_msg;

  // Build Message Layout
  resiliency_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  resiliency_msg.layout.dim[0].size = 8;
  resiliency_msg.layout.dim[0].stride = 1;
  // Publishing extra information:
  // cov_v_det and nrStIn should be the most relevant!
  resiliency_msg.layout.dim[0].label =
      "Values: cbrtPDet, cbrtVDet, nrStIn, nrMoIn. "
      "Thresholds : cbrtPDet, cbrtVDet, nrStIn, nrMoIn.";

  CHECK_EQ(pose_cov.size(), 36);
  gtsam::Matrix3 position_cov = gtsam::sub(pose_cov, 3, 6, 3, 6);
  CHECK_EQ(position_cov.size(), 9);

  // Compute eigenvalues and determinant of velocity covariance
  gtsam::Matrix U;
  gtsam::Matrix V;
  gtsam::Vector cov_v_eigv;
  gtsam::svd(vel_cov, U, cov_v_eigv, V);
  CHECK_EQ(cov_v_eigv.size(), 3);

  // Compute eigenvalues and determinant of position covariance
  gtsam::Vector cov_p_eigv;
  gtsam::svd(position_cov, U, cov_p_eigv, V);
  CHECK_EQ(cov_p_eigv.size(), 3);

  // Quality statistics to publish
  resiliency_msg.data.push_back(std::cbrt(cov_p_eigv(0)*
                    cov_p_eigv(1)*cov_p_eigv(2)));
  resiliency_msg.data.push_back(std::cbrt(cov_v_eigv(0)*
                    cov_v_eigv(1)*cov_v_eigv(2)));
  resiliency_msg.data.push_back(debug_tracker_info.nrStereoInliers_);
  resiliency_msg.data.push_back(debug_tracker_info.nrMonoInliers_);

  // Publish thresholds for statistics
  float pos_det_threshold, vel_det_threshold;
  int mono_ransac_theshold, stereo_ransac_threshold;
  CHECK(nh_.getParam("velocity_det_threshold", vel_det_threshold));
  CHECK(nh_.getParam("position_det_threshold", pos_det_threshold));
  CHECK(nh_.getParam("stereo_ransac_threshold", stereo_ransac_threshold));
  CHECK(nh_.getParam("mono_ransac_threshold", mono_ransac_theshold));\
  resiliency_msg.data.push_back(pos_det_threshold);
  resiliency_msg.data.push_back(vel_det_threshold);
  resiliency_msg.data.push_back(stereo_ransac_threshold);
  resiliency_msg.data.push_back(mono_ransac_theshold);

  // Publish Message
  resil_publisher_.publish(resiliency_msg);
}

void RosDataProvider::publishImuBias() {
  // Get imu bias to output
  ImuBias imu_bias = vio_output_.getEstimatedBias();
  Vector3 accel_bias = imu_bias.accelerometer();
  Vector3 gyro_bias = imu_bias.gyroscope();

  // Create message type
  std_msgs::Float64MultiArray imu_bias_msg;

  // Build Message Layout
  imu_bias_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  imu_bias_msg.layout.dim[0].size = 6;
  imu_bias_msg.layout.dim[0].stride = 1;
  imu_bias_msg.layout.dim[0].label = "Gyro Bias: x,y,z. Accel Bias: x,y,z";

  // Get Imu Bias to Publish
  imu_bias_msg.data.push_back(gyro_bias[0]);
  imu_bias_msg.data.push_back(gyro_bias[1]);
  imu_bias_msg.data.push_back(gyro_bias[2]);
  imu_bias_msg.data.push_back(accel_bias[0]);
  imu_bias_msg.data.push_back(accel_bias[1]);
  imu_bias_msg.data.push_back(accel_bias[2]);

  // Publish Message
  bias_publisher_.publish(imu_bias_msg);
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
  imuData_.print();
  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
}

} // namespace VIO
