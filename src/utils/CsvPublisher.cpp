#include "kimera_vio_ros/utils/CsvPublisher.h"

#include <glog/logging.h>

#include <kimera-vio/dataprovider/EurocDataProvider.h>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

namespace utils {

CsvPublisher::CsvPublisher()
    : nh_(), nh_private_("~"), vio_params_(""), euroc_data_provider_(nullptr) {
  CHECK(nh_private_.getParam("trajectory_label", trajectory_label_));
  CHECK(!trajectory_label_.empty())
      << "Please, specify a label for the parsed csv trajectory...";
  CHECK(nh_private_.getParam("world_frame_id", world_frame_id_));
  CHECK(!world_frame_id_.empty());

  // We assume Euroc csv format (and re-use the EurocDataProvider parser).
  euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(vio_params_);
  CHECK(euroc_data_provider_);
  // If parsed csv odometry is empty, nothing to do...
  if (euroc_data_provider_->gt_data_.map_to_gt_.empty()) {
    LOG(ERROR) << "CSV at " << euroc_data_provider_->getDatasetPath().c_str()
               << " is empty!";
    return;
  }
  euroc_data_provider_->gt_data_.print();
  csv_odometry_map_it_ = euroc_data_provider_->gt_data_.map_to_gt_.begin();

  // Init ROS publishers/subscribers
  static constexpr int queue_size = 100u;
  aligned_gt_odometry_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      trajectory_label_ + "_odometry", queue_size);
  aligned_gt_path_pub_ = nh_private_.advertise<nav_msgs::Path>(
      trajectory_label_ + "_path", queue_size);
  gt_path_pub_ = nh_private_.advertise<nav_msgs::Path>("gt_path", queue_size);
  vio_path_pub_ = nh_private_.advertise<nav_msgs::Path>("vio_path", queue_size);
  odometry_sub_ = nh_.subscribe(
      "input_odometry", queue_size, &CsvPublisher::callbackOdometry, this);
}

void CsvPublisher::callbackOdometry(
    const nav_msgs::OdometryConstPtr& odom_msg) {
  CHECK(euroc_data_provider_);
  const std::map<Timestamp, VioNavState>& csv_odometry_ =
      euroc_data_provider_->gt_data_.map_to_gt_;
  CHECK(!csv_odometry_.empty()) << "Parsed CSV odometry should not be empty...";

  const Timestamp& odometry_timestamp = odom_msg->header.stamp.toNSec();

  // If current VIO odometry is older than the oldest csv odometry,
  // nothing to do...
  if (odometry_timestamp > csv_odometry_.rbegin()->first) {
    return;
  }

  // Discard csv poses up to the current odometry timestamp or until we run out
  // of poses.
  while (csv_odometry_map_it_ != csv_odometry_.end() &&
         csv_odometry_map_it_->first < odometry_timestamp) {
    csv_odometry_map_it_++;
  }

  // If we finished with all CSV messages, return.
  if (csv_odometry_map_it_ == csv_odometry_.end()) {
    VLOG(1) << "Finished publishing CSV poses.";
    return;
  }

  // Gt pose expressed in Gt world frame of reference (gtw).
  gtsam::Pose3 gtw_P_gt = csv_odometry_map_it_->second.pose_;
  // VIO pose expressed in VIO world frame of reference (viow).
  gtsam::Pose3 viow_P_vio;
  rosOdometryToGtsamPose(*odom_msg, &viow_P_vio);
  gtsam::Vector3 gtw_V_gt = csv_odometry_map_it_->second.velocity_;

  // Discard first frames, while we wait for VIO to converge
  if (odometry_msgs_count_++ < n_discard_frames_) {
    VLOG(1) << "Discarding initial poses...";
    return;
  }

  // Align CSV odometry with VIO odometry for nice visualization
  // using the first n messages
  // TODO(Toni): if we were using an incremental method we might be better off
  // constantly aligning ground-truth with vio?
  if ((odometry_msgs_count_ - n_discard_frames_) < n_alignment_frames_) {
    // Associate VIO odometry with CSV (GT) odometry:
    VLOG(1) << "Adding measurement to align trajectories...";
    src_.conservativeResize(Eigen::NoChange, src_.cols() + 1);
    dst_.conservativeResize(Eigen::NoChange, dst_.cols() + 1);
    src_.col(src_.cols() - 1) = gtw_P_gt.translation();
    dst_.col(dst_.cols() - 1) = viow_P_vio.translation();
    return;
  }

  // Solve for R,t to align trajectories using Umeyama's method. (only once).
  if (!T_viow_gtw_) {
    CHECK_EQ(src_.rows(), dst_.rows());
    CHECK_EQ(src_.cols(), dst_.cols());
    VLOG(1) << "Calculating GT/VIO alignment transform...";
    static constexpr bool kSolveForScale = false;
    // Transforms from gt world frame to vio world frame.
    T_viow_gtw_ = std::make_shared<gtsam::Pose3>(
        Eigen::umeyama(src_, dst_, kSolveForScale));
    VLOG(1) << "Done calculating GT/VIO alignment transform: gt_T_vio_ = \n"
            << *T_viow_gtw_;
    CHECK_DOUBLE_EQ(T_viow_gtw_->rotation().toQuaternion().norm(), 1.0);
  } else {
    VLOG(1) << "Not re-calculating CSV/VIO initial alignment.";
  }
  CHECK(T_viow_gtw_);

  gtsam::Pose3 viow_P_gt = T_viow_gtw_->compose(gtw_P_gt);
  gtsam::Vector3 viow_V_gt = T_viow_gtw_->rotation().toQuaternion() * gtw_V_gt;

  // Publish Odometry
  nav_msgs::Odometry aligned_odometry;
  fillOdometryMsg(
      csv_odometry_map_it_->first, viow_P_gt, viow_V_gt, &aligned_odometry);
  aligned_gt_odometry_pub_.publish(aligned_odometry);

  nav_msgs::Odometry gt_odometry;
  fillOdometryMsg(csv_odometry_map_it_->first,
                  gtw_P_gt,
                  csv_odometry_map_it_->second.velocity_,
                  &gt_odometry);

  // Publish aligned Gt Path
  geometry_msgs::PoseStamped csv_pose_stamped;
  csv_pose_stamped.header = aligned_odometry.header;
  csv_pose_stamped.pose = aligned_odometry.pose.pose;
  aligned_gt_trajectory_path_.header = aligned_odometry.header;
  aligned_gt_trajectory_path_.poses.push_back(csv_pose_stamped);
  aligned_gt_path_pub_.publish(aligned_gt_trajectory_path_);

  // Publish regular Gt Path
  geometry_msgs::PoseStamped gt_pose_stamped;
  gt_pose_stamped.header = gt_odometry.header;
  gt_pose_stamped.pose = gt_odometry.pose.pose;
  gt_trajectory_path_.header = gt_odometry.header;
  gt_trajectory_path_.poses.push_back(gt_pose_stamped);
  gt_path_pub_.publish(gt_trajectory_path_);

  // Publish VIO Path
  geometry_msgs::PoseStamped vio_pose_stamped;
  vio_pose_stamped.header = odom_msg->header;
  vio_pose_stamped.pose = odom_msg->pose.pose;
  vio_trajectory_path_.header = odom_msg->header;
  vio_trajectory_path_.poses.push_back(vio_pose_stamped);
  vio_path_pub_.publish(vio_trajectory_path_);
}

void CsvPublisher::fillOdometryMsg(const Timestamp& timestamp,
                                   const gtsam::Pose3& pose,
                                   const gtsam::Vector3& velocity,
                                   nav_msgs::Odometry* odometry) const {
  CHECK_NOTNULL(odometry);
  odometry->header.stamp.fromNSec(timestamp);
  odometry->header.frame_id = world_frame_id_;
  odometry->child_frame_id = trajectory_label_;

  Eigen::Vector3d t = pose.translation();
  odometry->pose.pose.position.x = t.x();
  odometry->pose.pose.position.y = t.y();
  odometry->pose.pose.position.z = t.z();

  Eigen::Quaterniond R = pose.rotation().toQuaternion();
  odometry->pose.pose.orientation.w = R.w();
  odometry->pose.pose.orientation.x = R.x();
  odometry->pose.pose.orientation.y = R.y();
  odometry->pose.pose.orientation.z = R.z();

  odometry->twist.twist.linear.x = velocity.x();
  odometry->twist.twist.linear.y = velocity.y();
  odometry->twist.twist.linear.z = velocity.z();
}

}  // namespace utils

}  // namespace VIO

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "csv_publisher_node");
  VIO::utils::CsvPublisher csv_publisher;
  ros::spin();
}
