/**
 * @file   ros-data-source.h
 * @brief  ROS wrapper for online processing.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include "kimera_ros/RosDataProviderInterface.h"
#include "kimera_ros/stereo-image-buffer.h"

// using namespace StereoImageBuffer;

namespace VIO {

class RosOnlineDataProvider : public RosDataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosOnlineDataProvider);
  KIMERA_POINTER_TYPEDEFS(RosOnlineDataProvider);

  RosOnlineDataProvider();

  virtual ~RosOnlineDataProvider();

  bool spin() override;

  bool spinOnce();

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

  FrameId frame_count_left_;
  FrameId frame_count_right_;

  // Queues for fast input callbacks
  ThreadsafeQueue<sensor_msgs::ImuConstPtr> imu_input_queue_;
  ThreadsafeQueue<sensor_msgs::ImageConstPtr> left_camera_input_queue_;
  ThreadsafeQueue<sensor_msgs::ImageConstPtr> right_camera_input_queue_;

  // Reinitialization flag and packet (pose, vel, bias)
  bool reinit_flag_ = false;
  ReinitPacket reinit_packet_ = ReinitPacket();

 private:
  // Left camera callback
  void callbackLeftImage(const sensor_msgs::ImageConstPtr& msg);

  // Right camera callback
  void callbackRightImage(const sensor_msgs::ImageConstPtr& msg);

  // IMU callback
  void callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU);

  // Reinitialization callback
  void callbackReinit(const std_msgs::Bool::ConstPtr& reinitFlag);

  // Reinitialization pose
  void callbackReinitPose(const geometry_msgs::PoseStamped& reinitPose);

  // Message filters and to sync stereo images
  image_transport::Subscriber left_img_sub_;
  image_transport::Subscriber right_img_sub_;

  // Define subscriber for IMU data
  ros::Subscriber imu_subscriber_;

  // Define subscriber for Reinit data
  ros::Subscriber reinit_flag_subscriber_;
  ros::Subscriber reinit_pose_subscriber_;
};

}  // namespace VIO
