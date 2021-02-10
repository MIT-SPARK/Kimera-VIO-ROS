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
  odometry_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      trajectory_label_ + "_odometry", queue_size);
  path_pub_ = nh_private_.advertise<nav_msgs::Path>(trajectory_label_ + "_path",
                                                    queue_size);
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

  gtsam::Pose3 world_T_gt = csv_odometry_map_it_->second.pose_;
  gtsam::Pose3 world_T_vio;
  rosOdometryToGtsamPose(*odom_msg, &world_T_vio);

  // Align CSV odometry with VIO odometry for nice visualization
  // using the first n messages
  // TODO(Toni): if we were using an incremental method we might be better off
  // constantly aligning ground-truth with vio?
  if (odometry_msgs_count_ < n_alignment_frames_) {
    // Associate VIO odometry with CSV (GT) odometry:
    LOG(ERROR) << "Adding meas";
    src_.conservativeResize(src_.rows() + 1, Eigen::NoChange);
    dst_.conservativeResize(dst_.rows() + 1, Eigen::NoChange);
    src_.row(src_.rows() - 1) = world_T_gt.translation();
    dst_.row(dst_.rows() - 1) = world_T_vio.translation();
    odometry_msgs_count_++;
    return;
  }

  // Solve for R,t to align trajectories using Umeyama's method. (only once).
  if (!gt_T_vio_) {
    CHECK_EQ(src_.rows(), odometry_msgs_count_);
    CHECK_EQ(src_.rows(), dst_.rows());
    VLOG(1) << "Calculating GT/VIO alignment transform...";
    static constexpr bool kSolveForScale = false;
    Eigen::MatrixXd gt_T_vio = Eigen::umeyama(src_, dst_, kSolveForScale);
    gt_T_vio_ = std::make_shared<gtsam::Pose3>(gt_T_vio);
    VLOG(1) << "Done calculating GT/VIO alignment transform.";
  } else {
    VLOG(1) << "Not re-calculating CSV/VIO initial alignment.";
  }
  CHECK(gt_T_vio_);

  gtsam::Pose3 world_T_aligned_gt = world_T_gt.between(*gt_T_vio_);
  gtsam::Vector3 aligned_velocity = gt_T_vio_->rotation().toQuaternion() *
                                    csv_odometry_map_it_->second.velocity_;

  // Publish Odometry
  nav_msgs::Odometry csv_odometry;
  fillOdometryMsg(csv_odometry_map_it_->first,
                  world_T_aligned_gt,
                  aligned_velocity,
                  &csv_odometry);
  odometry_pub_.publish(csv_odometry);

  // Publish Path
  geometry_msgs::PoseStamped csv_pose_stamped;
  csv_pose_stamped.header = csv_odometry.header;
  csv_pose_stamped.pose = csv_odometry.pose.pose;
  csv_trajectory_path_.header = csv_odometry.header;
  csv_trajectory_path_.poses.push_back(csv_pose_stamped);
  path_pub_.publish(csv_trajectory_path_);
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
