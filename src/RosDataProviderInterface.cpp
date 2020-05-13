/**
 * @file   RosDataProviderInterface.cpp
 * @brief  Base class for ROS wrappers for Kimera-VIO.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/RosDataProviderInterface.h"

#include <string>
#include <vector>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/visualizer/Visualizer3D.h>

namespace VIO {

RosDataProviderInterface::RosDataProviderInterface(const VioParams& vio_params)
    : DataProviderInterface(),
      nh_(),
      nh_private_("~"),
      vio_params_(vio_params) {
  VLOG(1) << "Initializing RosDataProviderInterface.";

  // Print parameters to check.
  if (VLOG_IS_ON(1)) printParsedParams();
}

RosDataProviderInterface::~RosDataProviderInterface() {
  VLOG(1) << "RosBaseDataProvider destructor called.";
}

// TODO(marcus): From this documentation
//  (http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
//  we should be using toCvShare to get a CvImage pointer. However, this is a
//  shared pointer and the ROS message data isn't freed. This means anyone else
//  can modify this from another cb and our version will change too. Even the
//  const isn't enough because people can just copy that and still have access
//  to underlying data. Neet a smarter way to move these around yet make this
//  faster.
const cv::Mat RosDataProviderInterface::readRosImage(
    const sensor_msgs::ImageConstPtr& img_msg) const {
  CHECK(img_msg);
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  CHECK(cv_ptr);
  const cv::Mat img_const = cv_ptr->image;  // Don't modify shared image in ROS.
  cv::Mat converted_img(img_const.size(), CV_8U);
  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
    return converted_img;
  } else {
    CHECK_EQ(cv_ptr->encoding, sensor_msgs::image_encodings::MONO8)
        << "Expected image with MONO8, BGR8, or RGB8 encoding."
           "Add in here more conversions if you wish.";
    return img_const;
  }
}

const cv::Mat RosDataProviderInterface::readRosDepthImage(
    const sensor_msgs::ImageConstPtr& img_msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr =
        cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  cv::Mat img_depth = cv_ptr->image;
  if (img_depth.type() != CV_16UC1) {
    LOG_EVERY_N(WARNING, 10) << "Converting depth image...";
    img_depth.convertTo(img_depth, CV_16UC1);
  }
  return img_depth;
}

void RosDataProviderInterface::printParsedParams() const {
  static constexpr int kSeparatorWidth = 40;
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Left camera info:";
  vio_params_.camera_params_.at(0).print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Right camera info:";
  vio_params_.camera_params_.at(1).print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Frontend params:";
  vio_params_.frontend_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - IMU params:";
  vio_params_.imu_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Backend params";
  vio_params_.backend_params_->print();
  LOG(INFO) << std::string(kSeparatorWidth, '=');
  vio_params_.lcd_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=');
}

void RosDataProviderInterface::msgTFtoPose(const geometry_msgs::Transform& tf,
                                           gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);

  *pose = gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(
          tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)),
      gtsam::Point3(tf.translation.x, tf.translation.y, tf.translation.z));
}

void RosDataProviderInterface::poseToMsgTF(const gtsam::Pose3& pose,
                                           geometry_msgs::Transform* tf) {
  CHECK_NOTNULL(tf);

  tf->translation.x = pose.x();
  tf->translation.y = pose.y();
  tf->translation.z = pose.z();
  const gtsam::Quaternion& quat = pose.rotation().toQuaternion();
  tf->rotation.w = quat.w();
  tf->rotation.x = quat.x();
  tf->rotation.y = quat.y();
  tf->rotation.z = quat.z();
}

void RosDataProviderInterface::msgCamInfoToCameraParams(
    const sensor_msgs::CameraInfoConstPtr& cam_info,
    const std::string& cam_frame_id,
    VIO::CameraParams* cam_params) {
  CHECK_NOTNULL(cam_params);

  // Get intrinsics from incoming CameraInfo messages:
  cam_params->camera_id_ = cam_info->header.frame_id;
  CHECK(!cam_params->camera_id_.empty());

  cam_params->distortion_model_ = cam_info->distortion_model;
  CHECK(cam_params->distortion_model_ == "radtan" ||
        cam_params->distortion_model_ == "radial-tangential" ||
        cam_params->distortion_model_ == "equidistant");

  const std::vector<double>& distortion_coeffs = cam_info->D;
  CHECK_EQ(distortion_coeffs.size(), 4);
  VIO::CameraParams::convertDistortionVectorToMatrix(
      distortion_coeffs, &cam_params->distortion_coeff_);

  cam_params->image_size_ = cv::Size(cam_info->width, cam_info->height);

  cam_params->frame_rate_ = 0;  // TODO(marcus): is there a way to get this?

  std::array<double, 4> intrinsics = {
      cam_info->K[0], cam_info->K[4], cam_info->K[2], cam_info->K[5]};
  cam_params->intrinsics_ = intrinsics;
  VIO::CameraParams::convertIntrinsicsVectorToMatrix(cam_params->intrinsics_,
                                                     &cam_params->K_);

  VIO::CameraParams::createGtsamCalibration(cam_params->distortion_coeff_,
                                            cam_params->intrinsics_,
                                            &cam_params->calibration_);

  // Get extrinsics from the TF tree:
  tf2_ros::Buffer t_buffer;
  tf2_ros::TransformListener tf_listener(t_buffer);
  geometry_msgs::TransformStamped cam_tf;

  try {
    cam_tf = t_buffer.lookupTransform(base_link_frame_id_,
                                      cam_frame_id,
                                      ros::Time(0),
                                      ros::Duration(kTfLookupTimeout));
  } catch (tf2::TransformException& ex) {
    ROS_FATAL(
        "TF for left/right camera frames not available. Either publish to "
        "tree or provide CameraParameter yaml files.");
  }

  msgTFtoPose(cam_tf.transform, &cam_params->body_Pose_cam_);
}

}  // namespace VIO
