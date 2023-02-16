/**
 * @file   RosDataProviderInterface.h
 * @brief  Base class for ROS wrappers for Kimera-VIO.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <image_transport/subscriber_filter.h>
#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/logging/Logger.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <functional>
#include <opencv2/opencv.hpp>
#include <string>

#include "kimera_vio_ros/RosPublishers.h"

namespace VIO {

class RosDataProviderInterface : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDataProviderInterface);
  KIMERA_POINTER_TYPEDEFS(RosDataProviderInterface);

  explicit RosDataProviderInterface(const VioParams& vio_params);

  virtual ~RosDataProviderInterface();

 public:
  // virtual bool spin();

  void shutdown() override {
    DataProviderInterface::shutdown();
    LOG(INFO) << "RosDataProviderInterface shutdown.";
  }

  inline bool isShutdown() const { return shutdown_; }

 protected:
  const cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg) const;

  const cv::Mat readRosDepthImage(
      const sensor_msgs::ImageConstPtr& img_msg) const;

  void logGtData(const nav_msgs::OdometryConstPtr& odometry);

 protected:
  // Define Node Handler for general use (Parameter server)
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // TODO(Toni): technically the dataprovider should not need these, but
  // I still haven't removed the requirement to send camera params with each
  // vio callback...
  // Pipeline params
  VioParams vio_params_;

  bool log_gt_data_;
  bool is_header_written_poses_vio_;
  OfstreamWrapper::Ptr output_gt_poses_csv_;

 private:
  void printParsedParams() const;
};

}  // namespace VIO
