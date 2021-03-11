/**
 * @file   UtilsRos.cpp
 * @brief  Utilities to convert from/to Ros
 * @author Antoni Rosinol
 */

#include "kimera_vio_ros/utils/UtilsRos.h"

#include <string>

#include <glog/logging.h>

#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gtsam/geometry/Pose3.h>

#include <kimera-vio/common/VioNavState.h>
#include <kimera-vio/frontend/CameraParams.h>

namespace VIO {

namespace utils {

void rosTfToGtsamPose(const geometry_msgs::Transform& tf, gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);
  *pose = gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(
          tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)),
      gtsam::Point3(tf.translation.x, tf.translation.y, tf.translation.z));
}

void gtsamPoseToRosTf(const gtsam::Pose3& pose, geometry_msgs::Transform* tf) {
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

void rosOdometryToGtsamPose(const nav_msgs::Odometry& odom,
                            gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(odom.pose.pose.orientation.w,
                                                 odom.pose.pose.orientation.x,
                                                 odom.pose.pose.orientation.y,
                                                 odom.pose.pose.orientation.z);
  gtsam::Point3 translation(odom.pose.pose.position.x,
                            odom.pose.pose.position.y,
                            odom.pose.pose.position.z);
  *pose = gtsam::Pose3(rotation, translation);
}

void msgCamInfoToCameraParams(const sensor_msgs::CameraInfoConstPtr& cam_info,
                              const std::string& base_link_frame_id,
                              const std::string& cam_frame_id,
                              CameraParams* cam_params) {
  CHECK_NOTNULL(cam_params);

  // Get intrinsics from incoming CameraInfo messages:
  cam_params->camera_id_ = cam_info->header.frame_id;
  CHECK(!cam_params->camera_id_.empty());

  cam_params->distortion_model_ =
      CameraParams::stringToDistortion(cam_info->distortion_model, "pinhole");

  const std::vector<double>& distortion_coeffs = cam_info->D;
  CHECK_EQ(distortion_coeffs.size(), 4);
  cam_params->distortion_coeff_ = distortion_coeffs;
  CameraParams::convertDistortionVectorToMatrix(
      distortion_coeffs, &cam_params->distortion_coeff_mat_);

  cam_params->image_size_ = cv::Size(cam_info->width, cam_info->height);

  cam_params->frame_rate_ = 0;  // TODO(marcus): is there a way to get this?

  std::array<double, 4> intrinsics = {
      cam_info->K[0], cam_info->K[4], cam_info->K[2], cam_info->K[5]};
  cam_params->intrinsics_ = intrinsics;
  VIO::CameraParams::convertIntrinsicsVectorToMatrix(cam_params->intrinsics_,
                                                     &cam_params->K_);

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

  rosTfToGtsamPose(cam_tf.transform, &cam_params->body_Pose_cam_);
}

void rosOdometryToVioNavState(const nav_msgs::Odometry& odom,
                              const ros::NodeHandle& node_handle,
                              VioNavState* vio_navstate) {
  CHECK_NOTNULL(vio_navstate);

  rosOdometryToGtsamPose(odom, &vio_navstate->pose_);

  // World to Body rotation
  gtsam::Vector3 velocity(odom.twist.twist.linear.x,
                          odom.twist.twist.linear.y,
                          odom.twist.twist.linear.z);
  vio_navstate->velocity_ = velocity;

  // Get acceleration and gyro biases. Default is 0.
  std::vector<double> parsed_acc_bias = {0.0, 0.0, 0.0};
  node_handle.getParam("gt_accel_bias", parsed_acc_bias);
  CHECK_EQ(parsed_acc_bias.size(), 3u);

  std::vector<double> parsed_gyr_bias = {0.0, 0.0, 0.0};
  node_handle.getParam("gt_gyro_bias", parsed_gyr_bias);
  CHECK_EQ(parsed_gyr_bias.size(), 3u);

  gtsam::Vector3 acc_bias(
      parsed_acc_bias[0], parsed_acc_bias[1], parsed_acc_bias[2]);
  gtsam::Vector3 gyr_bias(
      parsed_gyr_bias[0], parsed_gyr_bias[1], parsed_gyr_bias[2]);

  vio_navstate->imu_bias_ = gtsam::imuBias::ConstantBias(acc_bias, gyr_bias);
}

}  // namespace utils

}  // namespace VIO
