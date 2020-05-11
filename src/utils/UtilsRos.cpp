/**
 * @file   UtilsRos.cpp
 * @brief  Utilities to convert from/to Ros
 * @author Antoni Rosinol
 */

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

namespace utils {

void msgTFtoPose(const geometry_msgs::Transform& tf, gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);

  *pose = gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(
          tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)),
      gtsam::Point3(tf.translation.x, tf.translation.y, tf.translation.z));
}

void poseToMsgTF(const gtsam::Pose3& pose, geometry_msgs::Transform* tf) {
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

void msgCamInfoToCameraParams(const sensor_msgs::CameraInfoConstPtr& cam_info,
                              const std::string& base_link_frame_id,
                              const std::string& cam_frame_id,
                              CameraParams* cam_params) {
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
  static constexpr size_t kTfLookupTimeout = 5u;
  geometry_msgs::TransformStamped cam_tf;
  try {
    cam_tf = t_buffer.lookupTransform(base_link_frame_id,
                                      cam_frame_id,
                                      ros::Time(0),
                                      ros::Duration(kTfLookupTimeout));
  } catch (tf2::TransformException& ex) {
    LOG(FATAL)
        << "TF for left/right camera frames not available. Either publish to "
           "tree or provide CameraParameter yaml files. Error: \n"
        << ex.what();
  }

  msgTFtoPose(cam_tf.transform, &cam_params->body_Pose_cam_);
}

}  // namespace utils

}  // namespace VIO
