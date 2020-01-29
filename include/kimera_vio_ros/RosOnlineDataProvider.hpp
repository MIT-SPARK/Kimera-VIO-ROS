/**
 * @file   ros-data-source.h
 * @brief  ROS wrapper for online processing.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#pragma once

#include "ros/callback_queue.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"


#include "kimera_vio_ros/RosDataProviderInterface.hpp"

namespace VIO {

class RosOnlineDataProvider : public RosDataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosOnlineDataProvider);
  KIMERA_POINTER_TYPEDEFS(RosOnlineDataProvider);

  RosOnlineDataProvider();

  virtual ~RosOnlineDataProvider();

  // Checks the current status of reinitialization flag
  inline bool getReinitFlag() const { return reinit_flag_; }
  // Resets the current status of reinitialization flag
  inline void resetReinitFlag() { reinit_packet_.resetReinitFlag(); }

 private:
  FrameId frame_count_;

  // Reinitialization flag and packet (pose, vel, bias)
  bool reinit_flag_ = false;
  ReinitPacket reinit_packet_ = ReinitPacket();

 private:
  // Left camera callback
  void callbackStereoImages(const sensor_msgs::msg::ImageConstPtr& left_msg,
                            const sensor_msgs::msg::ImageConstPtr& right_msg);

  // IMU callback
  void callbackIMU(const sensor_msgs::msg::ImuConstPtr& msgIMU);

  // GT odometry callback
  void callbackGtOdomOnce(const nav_msgs::msg::Odometry::ConstPtr& msgGtOdom);

  // Reinitialization callback
  void callbackReinit(const std_msgs::msg::Bool::ConstPtr& reinitFlag);

  // Reinitialization pose
  void callbackReinitPose(const geometry_msgs::msg::PoseStamped& reinitPose);

 private:
  void msgGtOdomToVioNavState(const nav_msgs::msg::Odometry::ConstPtr& gt_odom,
                              VioNavState* vio_navstate);

 private:
  // Message filters and to sync stereo images
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber left_img_subscriber_;
  ImageSubscriber right_img_subscriber_;

  // Declare Approx Synchronization Policy and Synchronizer for stereo images.
  // TODO(Toni): should be exact sync policy
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                          sensor_msgs::msg::Image>
      sync_pol;
  std::unique_ptr<message_filters::Synchronizer<sync_pol>> sync_;

  // Define subscriber for IMU data
  rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_;

  // Define subscriber for gt data
  ros::Subscriber gt_odom_subscriber_;

  // Define subscriber for Reinit data
  ros::Subscriber reinit_flag_subscriber_;
  ros::Subscriber reinit_pose_subscriber_;
};

}  // namespace VIO
