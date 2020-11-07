/**
 * @file   RosDataProviderInterface.h
 * @brief  Base class for ROS wrappers for Kimera-VIO.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <functional>
#include <string>

#include <opencv2/opencv.hpp>

#include <image_transport/subscriber_filter.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>

#include "kimera_vio_ros/RosPublishers.h"

namespace VIO {

class RosDataProviderInterface : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDataProviderInterface);
  KIMERA_POINTER_TYPEDEFS(RosDataProviderInterface);

  explicit RosDataProviderInterface(const VioParams& vio_params);

  virtual ~RosDataProviderInterface();

 public:
  //virtual bool spin();

  void shutdown() override {
    DataProviderInterface::shutdown();
    LOG(INFO) << "RosDataProviderInterface shutdown.";
  }


 protected:
  const cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg) const;

  const cv::Mat readRosDepthImage(
      const sensor_msgs::ImageConstPtr& img_msg) const;

 protected:
  // Define Node Handler for general use (Parameter server)
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // TODO(Toni): technically the dataprovider should not need these, but
  // I still haven't removed the requirement to send camera params with each
  // vio callback...
  // Pipeline params
  VioParams vio_params_;

 private:
  void printParsedParams() const;
};

}  // namespace VIO
