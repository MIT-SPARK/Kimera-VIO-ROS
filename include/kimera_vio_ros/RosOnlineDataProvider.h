/**
 * @file   RosOnlineDataProvider.h
 * @brief  ROS wrapper for online processing.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"

namespace VIO {

class RosOnlineDataProvider : public RosDataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosOnlineDataProvider);
  KIMERA_POINTER_TYPEDEFS(RosOnlineDataProvider);

  RosOnlineDataProvider(const VioParams& vio_params);

  virtual ~RosOnlineDataProvider();

 public:
  /**
   * @brief spin Runs the ros online data provider.
   * Parallel mode: starts async spinners, and returns true unless shutdown.
   * Sequential mode: calls ros::spinOnce
   * Then, both keep returning true unless shutdown.
   * @return True if nominal spin, false on shutdown.
   */
  bool spin() override;

  // Checks the current status of reinitialization flag
  inline bool getReinitFlag() const { return reinit_flag_; }
  // Resets the current status of reinitialization flag
  inline void resetReinitFlag() { reinit_packet_.resetReinitFlag(); }

 protected:
  /**
   * @brief parallelSpin Runs the dataprovider in parallel mode. It just
   * starts the asynchronous spinners and then returns true all the time.
   * @return True all the time.
   */
  bool parallelSpin();

  /**
   * @brief sequentialSpin Runs the dataprovider in sequential mode.
   * It calls the callbacks in the dedicated IMU queue and the rest of callbacks
   * (images) by using ros::spinOnce.
   * @return True all the time.
   */
  bool sequentialSpin();

 private:
  ros::CallbackQueue imu_queue_;
  std::unique_ptr<ros::AsyncSpinner> imu_async_spinner_;
  std::unique_ptr<ros::AsyncSpinner> async_spinner_;

  FrameId frame_count_;

  // Reinitialization flag and packet (pose, vel, bias)
  bool reinit_flag_ = false;
  ReinitPacket reinit_packet_ = ReinitPacket();

 private:
  // Stereo image callback
  void callbackStereoImages(const sensor_msgs::ImageConstPtr& left_msg,
                            const sensor_msgs::ImageConstPtr& right_msg);

  // CameraInfo callback
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& left_msg,
                          const sensor_msgs::CameraInfoConstPtr& right_msg);

  // IMU callback
  void callbackIMU(const sensor_msgs::ImuConstPtr& imu_msg);

  // GT odometry callback
  void callbackGtOdomOnce(const nav_msgs::Odometry::ConstPtr& gt_odom_msg);

  // Reinitialization callback
  void callbackReinit(const std_msgs::Bool::ConstPtr& reinitFlag);

  // Reinitialization pose
  void callbackReinitPose(const geometry_msgs::PoseStamped& reinitPose);

  // Publish static transforms (for camera frames) to the tf tree
  void publishStaticTf(const gtsam::Pose3& pose,
                       const std::string& parent_frame_id,
                       const std::string& child_frame_id);

 private:
  // Define image transport for this and derived classes.
  std::unique_ptr<image_transport::ImageTransport> it_;

  // Message filters and to sync stereo images
  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber left_img_subscriber_;
  ImageSubscriber right_img_subscriber_;

  // Define CameraInfo message subscribers
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo>
      CameraInfoSubscriber;
  CameraInfoSubscriber left_cam_info_subscriber_;
  CameraInfoSubscriber right_cam_info_subscriber_;

  // Declare Approx Synchronization Policy and Synchronizer for stereo images.
  // TODO(Toni): should be exact sync policy
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      sync_pol_img;
  typedef message_filters::sync_policies::
      ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
          sync_pol_info;
  std::unique_ptr<message_filters::Synchronizer<sync_pol_img>> sync_img_;
  std::unique_ptr<message_filters::Synchronizer<sync_pol_info>> sync_cam_info_;

  // Define subscriber for IMU data
  ros::Subscriber imu_subscriber_;

  // Define subscriber for gt data
  ros::Subscriber gt_odom_subscriber_;

  // Define subscriber for Reinit data
  ros::Subscriber reinit_flag_subscriber_;
  ros::Subscriber reinit_pose_subscriber_;

  // Ground-truth initialization pose received flag
  bool gt_init_pose_received_ = false;
  bool camera_info_received_ = false;

  // Have the async spinners start?
  bool started_async_spinners_ = false;

  // Frame ids
  std::string base_link_frame_id_;
  std::string left_cam_frame_id_;
  std::string right_cam_frame_id_;
};

}  // namespace VIO
