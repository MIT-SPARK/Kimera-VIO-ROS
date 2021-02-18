/**
 * @file   UtilsRos.h
 * @brief  Utilities to convert from/to ROS types
 * @author Antoni Rosinol
 */

#pragma once

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

void rosTfToGtsamPose(const geometry_msgs::Transform& tf, gtsam::Pose3* pose);

void gtsamPoseToRosTf(const gtsam::Pose3& pose, geometry_msgs::Transform* tf);

void rosOdometryToGtsamPose(const nav_msgs::Odometry& odom, gtsam::Pose3* pose);

void msgCamInfoToCameraParams(const sensor_msgs::CameraInfoConstPtr& cam_info,
                              const std::string& base_link_frame_id,
                              const std::string& cam_frame_id,
                              CameraParams* cam_params);

void rosOdometryToVioNavState(const nav_msgs::Odometry& gt_odom,
                              const ros::NodeHandle& node_handle,
                              VioNavState* vio_navstate);

}  // namespace utils

}  // namespace VIO
