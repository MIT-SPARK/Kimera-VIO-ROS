/**
 * @file   rosbag-data-source.cpp
 * @brief  Parse rosbad and run KimeraVIO
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#include "kimera_vio_ros/RosBagDataProvider.hpp"

#include <glog/logging.h>

#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>

namespace VIO {

RosbagDataProvider::RosbagDataProvider()
    : RosDataProviderInterface(),
      rosbag_data_(),
      rosbag_path_(""),
      left_imgs_topic_(""),
      right_imgs_topic_(""),
      imu_topic_(""),
      gt_odom_topic_(""),
      clock_pub_(),
      gt_odometry_pub_() {
  ROS_INFO("Starting KimeraVIO wrapper for offline");

  CHECK(nh_private_.getParam("rosbag_path", rosbag_path_));
  CHECK(nh_private_.getParam("left_cam_rosbag_topic", left_imgs_topic_));
  CHECK(nh_private_.getParam("right_cam_rosbag_topic", right_imgs_topic_));
  CHECK(nh_private_.getParam("imu_rosbag_topic", imu_topic_));
  CHECK(nh_private_.getParam("ground_truth_odometry_rosbag_topic",
                             gt_odom_topic_));

  CHECK(!rosbag_path_.empty());
  CHECK(!left_imgs_topic_.empty());
  CHECK(!right_imgs_topic_.empty());
  CHECK(!imu_topic_.empty());

  // Ros publishers specific to rosbag data provider
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);

  if (!gt_odom_topic_.empty()) {
    gt_odometry_pub_ =
        nh_.advertise<nav_msgs::Odometry>(gt_odom_topic_, 10);
  }
}

bool RosbagDataProvider::spin() {
  // Parse data from rosbag:
  CHECK(parseRosbag(rosbag_path_, &rosbag_data_));

  // Autoinitialize if necessary:
  if (pipeline_params_.backend_params_->autoInitialize_ == 0) {
    LOG(WARNING) << "Using initial ground-truth state for initialization.";
    pipeline_params_.backend_params_->initial_ground_truth_state_ =
        getGroundTruthVioNavState(0);  // Send first gt state.
  }

  // Send image data to VIO:
  Timestamp timestamp_last_frame = rosbag_data_.timestamps_.at(0);

  for (size_t k = 0; k < rosbag_data_.getNumberOfImages(); k++) {
    if (nh_.ok() && ros::ok() && !ros::isShuttingDown()) {
      // Main spin of the data provider: Interpolates IMU data
      // and builds StereoImuSyncPacket
      // (Think of this as the spin of the other parser/data-providers)
      const Timestamp& timestamp_frame_k = rosbag_data_.timestamps_.at(k);

      static const CameraParams& left_cam_info =
          pipeline_params_.camera_params_.at(0);
      static const CameraParams& right_cam_info =
          pipeline_params_.camera_params_.at(1);

      if (timestamp_frame_k > timestamp_last_frame) {
        // Send left frame data to Kimera.
        CHECK(left_frame_callback_)
            << "Did you forget to register the left frame callback?";
        left_frame_callback_(VIO::make_unique<Frame>(
            k,
            timestamp_frame_k,
            left_cam_info,
            readRosImage(rosbag_data_.left_imgs_.at(k))));

        // Send right frame data to Kimera.
        CHECK(right_frame_callback_)
            << "Did you forget to register the right frame callback?";
        right_frame_callback_(VIO::make_unique<Frame>(
            k,
            timestamp_frame_k,
            right_cam_info,
            readRosImage(rosbag_data_.right_imgs_.at(k))));

        // Publish ground-truth data if available
        if (rosbag_data_.gt_odometry_.size() > k) {
          publishGroundTruthOdometry(rosbag_data_.gt_odometry_.at(k));
        }

        VLOG(10) << "Finished VIO processing for frame k = " << k;

        // Publish VIO output if any.
        // TODO(Toni) this could go faster if running in another thread or
        // node...
        bool got_synced_outputs = publishSyncedOutputs();
        if (!got_synced_outputs) {
          LOG(WARNING) << "Pipeline lagging behind rosbag parser.";
        }

        // Publish LCD output if any.
        LcdOutput::Ptr lcd_output = nullptr;
        if (lcd_output_queue_.pop(lcd_output)) {
          publishLcdOutput(lcd_output);
        }

        timestamp_last_frame = timestamp_frame_k;
      } else {
        ROS_WARN(
            "Skipping frame %d. Frame timestamps out of order:"
            " less than or equal to last frame.",
            static_cast<int>(k));
      }

      ros::spinOnce();
    } else {
      LOG(ERROR) << "ROS SHUTDOWN requested, stopping rosbag spin.";
      ros::shutdown();
      return false;
    }
  }  // End of for loop over rosbag images.
  LOG(INFO) << "Rosbag processing finished.";

  // Endless loop until ros dies to publish left-over outputs.
  while (nh_.ok() && ros::ok() && !ros::isShuttingDown()) {
    publishSyncedOutputs();

    LcdOutput::Ptr lcd_output = nullptr;
    if (lcd_output_queue_.pop(lcd_output)) {
      publishLcdOutput(lcd_output);
    }
  }

  return true;
}

bool RosbagDataProvider::parseRosbag(const std::string& bag_path,
                                     RosbagData* rosbag_data) {
  LOG(INFO) << "Parsing rosbag data.";
  CHECK_NOTNULL(rosbag_data);

  // Fill in rosbag to data_
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  // Generate list of topics to parse:
  std::vector<std::string> topics;
  topics.push_back(left_imgs_topic_);
  topics.push_back(right_imgs_topic_);
  topics.push_back(imu_topic_);
  if (!gt_odom_topic_.empty()) {
    CHECK(pipeline_params_.backend_params_->autoInitialize_ == 0)
        << "Provided a gt_odom_topic; but autoInitialize is not set to 0,"
           " meaning no ground-truth initialization will be done... "
           "Are you sure you don't want to use gt? ";
    topics.push_back(gt_odom_topic_);
  } else {
    CHECK(pipeline_params_.backend_params_->autoInitialize_ != 0);
    ROS_DEBUG("Not parsing ground truth data.");
  }

  // Query rosbag for given topics
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Keep track of this since we expect IMU data before an image.
  bool start_parsing_stereo = false;
  // For some datasets, we have duplicated measurements for the same time.
  Timestamp last_imu_timestamp = 0;
  for (const rosbag::MessageInstance& msg : view) {
    // Get topic.
    const std::string& msg_topic = msg.getTopic();

    // IMU
    sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
    if (imu_msg != nullptr && msg_topic == imu_topic_) {
      ImuAccGyr imu_accgyr;
      imu_accgyr(0) = imu_msg->linear_acceleration.x;
      imu_accgyr(1) = imu_msg->linear_acceleration.y;
      imu_accgyr(2) = imu_msg->linear_acceleration.z;
      imu_accgyr(3) = imu_msg->angular_velocity.x;
      imu_accgyr(4) = imu_msg->angular_velocity.y;
      imu_accgyr(5) = imu_msg->angular_velocity.z;
      const ImuStamp& imu_data_timestamp = imu_msg->header.stamp.toNSec();
      if (imu_data_timestamp > last_imu_timestamp) {
        // Send IMU data directly to VIO at parse level for speed boost:
        CHECK(imu_single_callback_)
            << "Did you forget to register the IMU callback?";
        imu_single_callback_(ImuMeasurement(imu_data_timestamp, imu_accgyr));

        last_imu_timestamp = imu_data_timestamp;
      } else {
        ROS_FATAL(
            "IMU timestamps in rosbag are out of order: consider re-ordering "
            "rosbag.");
      }
      start_parsing_stereo = true;
    } else {
      // Check if msg is an image.
      sensor_msgs::ImageConstPtr img = msg.instantiate<sensor_msgs::Image>();
      if (img != nullptr) {
        if (start_parsing_stereo) {
          // Check left or right image.
          if (msg_topic == left_imgs_topic_) {
            // Timestamp is in nanoseconds
            rosbag_data->timestamps_.push_back(img->header.stamp.toNSec());
            rosbag_data->left_imgs_.push_back(img);
          } else if (msg_topic == right_imgs_topic_) {
            rosbag_data->right_imgs_.push_back(img);
          } else {
            ROS_WARN_STREAM("Img with unexpected topic: " << msg_topic);
          }
        } else {
          ROS_WARN(
              "Skipping first frame in rosbag, since IMU data not yet "
              "available.");
        }
      } else {
        nav_msgs::OdometryConstPtr gt_odometry =
            msg.instantiate<nav_msgs::Odometry>();
        if (gt_odometry != nullptr) {
          if (msg_topic == gt_odom_topic_) {
            rosbag_data->gt_odometry_.push_back(gt_odometry);
          } else {
            ROS_ERROR(
                "Unrecognized topic name for odometry msg. We were"
                " expecting ground-truth odometry on this topic.");
          }
        } else {
          ROS_ERROR_STREAM(
              "Could not find the type of this rosbag msg from topic:\n"
              << msg.getTopic());
        }
      }
    }
  }
  bag.close();

  // Sanity check:
  ROS_ERROR_COND(rosbag_data->left_imgs_.size() == 0 ||
                     rosbag_data->right_imgs_.size() == 0,
                 "No images parsed from rosbag!");
  // Without saving imu data offline, we can't perform this check.
  // ROS_ERROR_COND(imu_data_.imu_buffer_.size() <=
  // rosbag_data->left_imgs_.size(),
  //                "Less than or equal number fo imu data as image data.");
  ROS_ERROR_COND(
      !gt_odom_topic_.empty() && rosbag_data_.gt_odometry_.size() > 0,
      "Requested to parse ground-truth odometry, but parsed 0 msgs.");
  ROS_ERROR_COND(!gt_odom_topic_.empty() && rosbag_data->gt_odometry_.size() !=
                                                rosbag_data->left_imgs_.size(),
                 "Different number of ground_truth data than image data.");
  return true;
}

VioNavState RosbagDataProvider::getGroundTruthVioNavState(
    const size_t& k_frame) const {
  CHECK_LE(k_frame, rosbag_data_.gt_odometry_.size());
  nav_msgs::Odometry gt_odometry = *(rosbag_data_.gt_odometry_.at(k_frame));
  // World to Body rotation
  gtsam::Rot3 W_R_B =
      gtsam::Rot3::Quaternion(gt_odometry.pose.pose.orientation.w,
                              gt_odometry.pose.pose.orientation.x,
                              gt_odometry.pose.pose.orientation.y,
                              gt_odometry.pose.pose.orientation.z);
  gtsam::Point3 position(gt_odometry.pose.pose.position.x,
                         gt_odometry.pose.pose.position.y,
                         gt_odometry.pose.pose.position.z);
  gtsam::Vector3 velocity(gt_odometry.twist.twist.linear.x,
                          gt_odometry.twist.twist.linear.y,
                          gt_odometry.twist.twist.linear.z);
  VioNavState gt_init;
  gt_init.pose_ = gtsam::Pose3(W_R_B, position);
  gt_init.velocity_ = velocity;
  // TODO(Toni): how can we get the ground-truth biases? For sim, ins't it 0?
  gtsam::Vector3 gyro_bias(0.0, 0.0, 0.0);
  gtsam::Vector3 acc_bias(0.0, 0.0, 0.0);
  gt_init.imu_bias_ = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
  return gt_init;
}

void RosbagDataProvider::publishBackendOutput(
    const BackendOutput::Ptr& output) {
  RosDataProviderInterface::publishBackendOutput(output);
  publishClock(output->timestamp_);
}

void RosbagDataProvider::publishClock(const Timestamp& timestamp) const {
  rosgraph_msgs::Clock clock;
  clock.clock.fromNSec(timestamp);
  clock_pub_.publish(clock);
}

void RosbagDataProvider::publishGroundTruthOdometry(
    const nav_msgs::OdometryConstPtr& gt_odom) const {
  gt_odometry_pub_.publish(gt_odom);
}

}  // namespace VIO
