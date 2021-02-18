/**
 * @file   CsvPublisher.h
 * @brief  Publishes a 3D trajectory provided as a CSV file (Euroc format)
 * at the same pace than a given odometry topic, and aligns the CSV trajectory
 * with the odometry.
 * This is nice to visualize a ground-truth trajectory displayed at the same
 * time that we get
 * VIO's results so that we can compare both trajectories visually.
 * @author Antoni Rosinol
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <kimera-vio/utils/Macros.h>
#include <kimera-vio/common/vio_types.h>
#include <kimera-vio/common/VioNavState.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/dataprovider/EurocDataProvider.h>

namespace VIO {

class EurocDataProvider;

namespace utils {

class CsvPublisher {
 public:
  KIMERA_POINTER_TYPEDEFS(CsvPublisher);
  KIMERA_DELETE_COPY_CONSTRUCTORS(CsvPublisher);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using MapIterator = std::map<Timestamp, VioNavState>::iterator;

 public:
  /**
  * @brief CsvPublisher
  * The path to the csv dataset published is given by the dataset_path gflag.
  */
  CsvPublisher();
  virtual ~CsvPublisher() = default;

 protected:
  void callbackOdometry(const nav_msgs::OdometryConstPtr& odom_msg);

  void fillOdometryMsg(const Timestamp& timestamp,
                       const gtsam::Pose3& pose,
                       const gtsam::Vector3& velocity,
                       nav_msgs::Odometry* odometry) const;

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher aligned_gt_odometry_pub_;
  ros::Publisher aligned_gt_path_pub_;
  ros::Publisher gt_path_pub_;
  ros::Publisher vio_path_pub_;

  ros::Subscriber odometry_sub_;

  nav_msgs::Path aligned_gt_trajectory_path_;
  nav_msgs::Path gt_trajectory_path_;
  nav_msgs::Path vio_trajectory_path_;

  /// This label is prepended to the published topics and used as
  /// the child frame id for the published odometry.
  std::string trajectory_label_ = "aligned_gt";
  std::string world_frame_id_ = "world";

  VioParams vio_params_;
  EurocDataProvider::UniquePtr euroc_data_provider_;

  MapIterator csv_odometry_map_it_;

  Eigen::Matrix3Xd src_;
  Eigen::Matrix3Xd dst_;
  std::shared_ptr<gtsam::Pose3> T_viow_gtw_ = nullptr;

  // Align trajectories
  static constexpr int n_alignment_frames_ = 30;
  static constexpr int n_discard_frames_ = 20;
  int odometry_msgs_count_ = 0;
};

}  // namespace utils

}  // namespace VIO
