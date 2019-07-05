/**
 * @file   ros-base-data-source.h
 * @brief  ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include <functional>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <ImuFrontEnd.h>
#include <StereoFrame.h>
#include <StereoImuSyncPacket.h>
#include <VioFrontEndParams.h>
#include <common/vio_types.h>
#include <datasource/DataSource.h>
#include <pipeline/Pipeline.h>

#include "spark-vio-ros/stereo-image-buffer.h"

namespace VIO {

struct RosbagData {
  inline size_t getNumberOfImages() const { return left_imgs_.size(); }
  // The names of the images from left camera
  std::vector<sensor_msgs::ImageConstPtr> left_imgs_;
  // The names of the images from right camera
  std::vector<sensor_msgs::ImageConstPtr> right_imgs_;
  // Vector of timestamps see issue in .cpp file
  std::vector<Timestamp> timestamps_;
  // IMU data
  ImuData imu_data_;
};

class RosBaseDataProvider : public DataProvider {
 public:
  RosBaseDataProvider();
  virtual ~RosBaseDataProvider();

 protected:
  // Define Node Handler for general use (Parameter server)
  ros::NodeHandle nh_;

  // Stereo info
  struct StereoCalibration {
    CameraParams left_camera_info_;
    CameraParams right_camera_info_;
    gtsam::Pose3 camL_Pose_camR_;  // relative pose between cameras
  };

 protected:
  cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg);

  cv::Mat readRosRGBImage(const sensor_msgs::ImageConstPtr& img_msg);

  cv::Mat readRosDepthImage(const sensor_msgs::ImageConstPtr& img_msg);

  // Parse camera calibration info (from param server)
  bool parseCameraData(StereoCalibration* stereo_calib);

  // Parse IMU calibration info (for ros online)
  bool parseImuData(ImuData* imudata, ImuParams* imuparams);

  // Parse IMU calibration info (for rosbag)
  bool parseImuData(RosbagData* rosbag_data, ImuParams* imuparams);

  // Publish all outputs by calling individual functions below
  void publishOutput();

  // Publish current state estimate
  void publishState();

  // Publish frontend statistics
  void publishFrontendStats();

  // Publish resiliency statistics
  void publishResiliency();

  // Publish IMU bias
  void publishImuBias();

  // Define publisher to publish odometry
  ros::Publisher odom_publisher_;

  // Define publisher to publish resiliency
  ros::Publisher resil_publisher_;

  // Define frontend statistics publisher
  ros::Publisher frontend_stats_publisher_;

  // Define publisher to publish imu bias
  ros::Publisher bias_publisher_;

  // Define tf broadcaster for world to base_link (IMU).
  tf::TransformBroadcaster odom_broadcaster_;

  // Define frame ids for odometry message
  std::string odom_base_frame_id_;
  std::string odom_child_frame_id_;

 protected:
  VioFrontEndParams frontend_params_;
  StereoCalibration stereo_calib_;
  SpinOutputContainer vio_output_;
};

}  // namespace VIO
