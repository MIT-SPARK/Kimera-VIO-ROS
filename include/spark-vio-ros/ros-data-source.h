/**
 * @file   ros-data-source.h
 * @brief  ROS wrapper for online processing.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#pragma once

#include "spark-vio-ros/base-data-source.h"
#include "spark-vio-ros/stereo-image-buffer.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

// using namespace StereoImageBuffer;

namespace VIO {

class RosDataProvider : public RosBaseDataProvider {
 public:
  RosDataProvider();
  virtual ~RosDataProvider();
  virtual bool spin() override;

  // Checks the current status of reinitialization flag
  inline bool getReinitFlag() const { return reinit_flag_; }
  // Resets the current status of reinitialization flag
  void resetReinitFlag() { reinit_packet_.resetReinitFlag(); }

 private:
  // TODO (Toni): only use one node handle...
  // Define Node Handler for IMU Callback (and Queue)
  ros::NodeHandle nh_imu_;

  // Define Node Handler for Reinit Callback (and Queue)
  ros::NodeHandle nh_reinit_;

  // Define Node Handler for Cam Callback (and Queue)
  ros::NodeHandle nh_cam_;

  ImuData imu_data_;               // store IMU data from last frame
  Timestamp last_time_stamp_;      // Timestamp correponding to last frame
  Timestamp last_imu_time_stamp_;  // Timestamp corresponding to last imu meas
  int frame_count_;                // Keep track of number of frames processed

  StereoBuffer stereo_buffer_;

  // Reinitialization flag and packet (pose, vel, bias)
  bool reinit_flag_ = false;
  ReinitPacket reinit_packet_ = ReinitPacket();

 private:
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
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber left_img_subscriber_;
  ImageSubscriber right_img_subscriber_;

  // Declare Approx Synchronization Policy and Synchronizer for stereo images.
  // TODO(Toni): should be exact sync policy
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol;
  std::unique_ptr<message_filters::Synchronizer<sync_pol>> sync_;

  // Define subscriber for IMU data
  ros::Subscriber imu_subscriber_;

  // Define subscriber for Reinit data
  ros::Subscriber reinit_flag_subscriber_;
  ros::Subscriber reinit_pose_subscriber_;

  // Print the parameters
  void print() const;
};

}  // namespace VIO
