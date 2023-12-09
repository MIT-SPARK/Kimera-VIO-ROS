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
      vio_params_(vio_params),
      is_header_written_poses_vio_(false),
      log_gt_data_(false) {
  VLOG(1) << "Initializing RosDataProviderInterface.";

  CHECK(nh_private_.getParam("log_gt_data", log_gt_data_));
  if (log_gt_data_) {
    output_gt_poses_csv_.reset(new OfstreamWrapper("traj_gt.csv"));
  }

  if (VLOG_IS_ON(1)) printParsedParams();  // Print parameters to check.
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
    VLOG_EVERY_N(1, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    VLOG_EVERY_N(1, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
    VLOG_EVERY_N(1, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGRA2GRAY);
    return converted_img;
  } else {
    CHECK(cv_ptr->encoding == sensor_msgs::image_encodings::MONO8 ||
          cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
        << "Expected image with MONO8, 8UC1, BGR8, or RGB8 encoding."
           "Add in here more conversions if you wish.";
    return img_const;
  }
}

const cv::Mat RosDataProviderInterface::readRosDepthImage(
    const sensor_msgs::ImageConstPtr& img_msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  cv::Mat img_depth = cv_ptr->image;
  CHECK_EQ(img_depth.channels(), 1) << "Depth image must have a single channel";

  if (img_depth.type() != CV_32FC1 && img_depth.type() != CV_16UC1) {
    LOG_FIRST_N(WARNING, 10)
        << "Converting depth image to CV_32FC1. Consider supplementing "
           "DepthFrame if this causes loss of precision";
    // floating point depth makes things easier later
    img_depth.convertTo(img_depth, CV_32FC1);
  }
  return img_depth;
}

void RosDataProviderInterface::logGtData(
    const nav_msgs::OdometryConstPtr& odometry) {
  CHECK(odometry);
  if (!output_gt_poses_csv_) {
    LOG(ERROR) << "GT pose file not initialized!";
    return;
  }

  // We log the poses in csv format for later alignement and analysis.
  std::ofstream& output_stream = output_gt_poses_csv_->ofstream_;
  bool& is_header_written = is_header_written_poses_vio_;

  // First, write header, but only once.
  if (!is_header_written) {
    output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,"
                  << "bgx,bgy,bgz,bax,bay,baz" << std::endl;
    is_header_written = true;
  }
  output_stream << odometry->header.stamp.toNSec() << ","  //
                << odometry->pose.pose.position.x << ","   //
                << odometry->pose.pose.position.y << ","   //
                << odometry->pose.pose.position.z << ","   //
                << odometry->pose.pose.orientation.w << ","
                << odometry->pose.pose.orientation.x << ","
                << odometry->pose.pose.orientation.y << ","
                << odometry->pose.pose.orientation.z << ","
                << odometry->twist.twist.linear.x << ","
                << odometry->twist.twist.linear.y << ","
                << odometry->twist.twist.linear.z << "," << 0.0 << ","  //
                << 0.0 << ","                                           //
                << 0.0 << ","                                           //
                << 0.0 << ","                                           //
                << 0.0 << ","                                           //
                << 0.0                                                  //
                << std::endl;
}

void RosDataProviderInterface::printParsedParams() const {
  static constexpr int kSeparatorWidth = 40;
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Left camera info:";
  vio_params_.camera_params_.at(0).print();
  if (vio_params_.camera_params_.size() > 1) {
    LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Right camera info:";
    vio_params_.camera_params_.at(1).print();
  }
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

}  // namespace VIO
