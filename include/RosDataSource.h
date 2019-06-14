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

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>

#include <common/vio_types.h>
#include <datasource/DataSource.h>
#include <StereoImuSyncPacket.h>
#include <StereoFrame.h>
#include <VioFrontEndParams.h>
#include <ImuFrontEnd.h>
#include <ETH_parser.h>
#include <pipeline/Pipeline.h>

#include "StereoImageBuffer.h"

// using namespace StereoImageBuffer;

namespace VIO {

class RosDataProvider: public DataProvider {
public:
  RosDataProvider(std::string left_camera_topic,
                  std::string right_camera_topic,
                  std::string imu_topic,
                  std::string reinit_flag_topic,
                  std::string reinit_pose_topic);
  virtual ~RosDataProvider();
  virtual bool spin();

  inline ImuParams getImuParams() const {
    return imuParams_;
  }

  // Checks the current status of reinitialization flag
  inline bool getReinitFlag() const { return reinit_flag_;}
  // Resets the current status of reinitialization flag
  void resetReinitFlag() { reinit_packet_.resetReinitFlag();}

private:

  // Define Node Handler for general use (Parameter server)
  ros::NodeHandle nh_;

  // Define Node Handler for IMU Callback (and Queue)
  ros::NodeHandle nh_imu_;

  // Define Node Handler for Reinit Callback (and Queue)
  ros::NodeHandle nh_reinit_;

  // Define Node Handler for Cam Callback (and Queue)
  ros::NodeHandle nh_cam_;
  image_transport::ImageTransport it_;
  // image_tranport should be used for images instead of subscribers

  typedef image_transport::SubscriberFilter ImageSubscriber;

  ImuData imuData_; // store IMU data from last frame
  ImuParams imuParams_;
  Timestamp last_time_stamp_; // Timestamp correponding to last frame
  Timestamp last_imu_time_stamp_; // Timestamp corresponding to last imu meas
  int frame_count_; // Keep track of number of frames processed

  StereoBuffer stereo_buffer_;

  // Reinitialization flag and packet (pose, vel, bias)
  bool reinit_flag_ = false;
  ReinitPacket reinit_packet_ = ReinitPacket();

  // Stereo info
  struct StereoCalibration{
    CameraParams left_camera_info_;
    CameraParams right_camera_info_;
    gtsam::Pose3 camL_Pose_camR_; // relative pose between cameras
  };

private:
  cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg);

  cv::Mat readRosRGBImage(const sensor_msgs::ImageConstPtr& img_msg);

  cv::Mat readRosDepthImage(const sensor_msgs::ImageConstPtr& img_msg);

  // Parse camera calibration info (from param server)
  bool parseCameraData(StereoCalibration* stereo_calib);

  // Parse IMU calibration info
  bool parseImuData(ImuData* imudata, ImuParams* imuparams);

  // IMU callback
  void callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU);

  // Reinitialization callback
  void callbackReinit(const std_msgs::Bool::ConstPtr& reinitFlag);

  // Reinitialization pose
  void callbackReinitPose(const geometry_msgs::PoseStamped& reinitPose);

  // Callback for stereo images and main spin
  void callbackCamAndProcessStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                                   const sensor_msgs::ImageConstPtr& msgRight);

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

  // Define subscriber for Reinit data
  ros::Subscriber reinit_flag_subscriber_;
  ros::Subscriber reinit_pose_subscriber_;

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

  // Define imu topic since might need to wait
  std::string imu_topic_;

  // Define reinitialization topic
  std::string reinit_flag_topic_;
  std::string reinit_pose_topic_;

  // Print the parameters
  void print() const;

private:
  VioFrontEndParams frontend_params_;
  StereoCalibration stereo_calib_;
  SpinOutputContainer vio_output_;
};

} // End of VIO Namespace
