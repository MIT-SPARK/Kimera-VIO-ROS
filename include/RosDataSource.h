/**
 * @file   RosDataSource.h
 * @brief  ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include "RosBaseDataSource.h"

#include "StereoImageBuffer.h"

// using namespace StereoImageBuffer;

namespace VIO {

class RosDataProvider: public RosBaseDataProvider {
public:
  RosDataProvider(std::string left_camera_topic,
                  std::string right_camera_topic,
                  std::string imu_topic,
                  std::string reinit_flag_topic,
                  std::string reinit_pose_topic);
  virtual ~RosDataProvider();
  bool spin();

  // Checks the current status of reinitialization flag
  inline bool getReinitFlag() const { return reinit_flag_;}
  // Resets the current status of reinitialization flag
  void resetReinitFlag() { reinit_packet_.resetReinitFlag();}

private:

  // Define Node Handler for IMU Callback (and Queue)
  ros::NodeHandle nh_imu_;

  // Define Node Handler for Reinit Callback (and Queue)
  ros::NodeHandle nh_reinit_;

  // Define Node Handler for Cam Callback (and Queue)
  ros::NodeHandle nh_cam_;
  image_transport::ImageTransport it_;
  // image_tranport should be used for images instead of subscribers

  typedef image_transport::SubscriberFilter ImageSubscriber;

  ImuData imu_data_; // store IMU data from last frame
  Timestamp last_time_stamp_; // Timestamp correponding to last frame
  Timestamp last_imu_time_stamp_; // Timestamp corresponding to last imu meas
  int frame_count_; // Keep track of number of frames processed

  StereoBuffer stereo_buffer_;

  // Reinitialization flag and packet (pose, vel, bias)
  bool reinit_flag_ = false;
  ReinitPacket reinit_packet_ = ReinitPacket();

protected:
  // IMU callback
  void callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU);

  // Reinitialization callback
  void callbackReinit(const std_msgs::Bool::ConstPtr& reinitFlag);

  // Reinitialization pose
  void callbackReinitPose(const geometry_msgs::PoseStamped& reinitPose);

  // Callback for stereo images and main spin
  void callbackCamAndProcessStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                                   const sensor_msgs::ImageConstPtr& msgRight);

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

  // Define reinitialization topic
  std::string reinit_flag_topic_;
  std::string reinit_pose_topic_;

  // Print the parameters
  void print() const;
};

} // End of VIO Namespace
