/**
 * @file   RosDataSource.h
 * @brief  ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include <string>
#include <functional>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include "datasource/DataSource.h"
#include "StereoImuSyncPacket.h"
#include "StereoFrame.h"
#include "VioFrontEndParams.h"
#include "ImuFrontEnd.h"
#include "ETH_parser.h"
#include "pipeline/Pipeline.h"

// ROS Dependencies
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/CameraInfo.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "StereoImageBuffer.h"

// using namespace StereoImageBuffer;

namespace VIO {

class RosDataProvider: public DataProvider {
public:
  RosDataProvider(std::string left_camera_topic,
                  std::string right_camera_topic,
                  std::string imu_topic);
  virtual ~RosDataProvider();
  virtual bool spin();

  inline ImuParams getImuParams() const {
    return imuParams_;
  }

private:

  // Define Node Handler for general use (Parameter server)
  ros::NodeHandle nh_;

  // Define Node Handler for IMU Callback (and Queue)
  ros::NodeHandle nh_imu_;

  // Define Node Handler for Cam Callback (and Queue)
  ros::NodeHandle nh_cam_;
  image_transport::ImageTransport it_;
  // image_tranport should be used for images instead of subscribers

  typedef image_transport::SubscriberFilter ImageSubscriber;

  ImuData imuData_; // store IMU data from last frame
  ImuParams imuParams_;
  Timestamp last_time_stamp_; // Timestamp correponding to last frame
  int frame_count_; // Keep track of number of frames processed

  StereoBuffer stereo_buffer_;

  // Stereo info
  struct StereoCalibration{
    CameraParams left_camera_info_;
    CameraParams right_camera_info_;
    gtsam::Pose3 camL_Pose_camR_; // relative pose between cameras
  };

private:
  cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg);

  // Parse camera calibration info (from param server)
  bool parseCameraData(StereoCalibration* stereo_calib);

  // Parse IMU calibration info
  bool parseImuData(ImuData* imudata, ImuParams* imuparams);

  // IMU callback
  void callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU);

  // Callback for stereo images and main spin
  void callbackCamAndProcessStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                                   const sensor_msgs::ImageConstPtr& msgRight);

  void publishOutput(gtsam::Pose3 pose, gtsam::Vector3 velocity, Timestamp ts) const;

  // Message filters and to sync stereo images
  ImageSubscriber left_img_subscriber_;
  ImageSubscriber right_img_subscriber_;

  // Declare Synchronization Policy for Stereo
  typedef message_filters::sync_policies::ApproximateTime
  <sensor_msgs::Image, sensor_msgs::Image> sync_pol;

  // Declare synchronizer
  message_filters::Synchronizer<sync_pol> sync;

  // Define subscriber for IMU data
  ros::Subscriber imu_subscriber_;

  // Define publisher to publish odometry
  ros::Publisher odom_publisher;

  // Define imu topic since might need to wait
  std::string imu_topic_;

  // Print the parameters
  void print() const;

private:
  VioFrontEndParams frontend_params_;
  StereoCalibration stereo_calib_;
};

} // End of VIO Namespace
