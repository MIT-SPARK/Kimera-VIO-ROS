/**
 * @file   UtilsRos.h
 * @brief  Utilities to convert from/to ROS types
 * @author Antoni Rosinol
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/CameraInfo.h>

#include <kimera-vio/frontend/CameraParams.h>

namespace VIO {

namespace utils {

namespace gtsam {
class Pose3;
}

void msgTFtoPose(const geometry_msgs::Transform& tf, gtsam::Pose3* pose);

void poseToMsgTF(const gtsam::Pose3& pose, geometry_msgs::Transform* tf);

void msgCamInfoToCameraParams(const sensor_msgs::CameraInfoConstPtr& cam_info,
                              const std::string& cam_frame_id,
                              CameraParams* cam_params);
}  // namespace utils

}  // namespace VIO
