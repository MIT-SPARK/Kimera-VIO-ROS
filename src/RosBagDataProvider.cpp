/**
 * @file   RosBagDataProvider.cpp
 * @brief  Parse rosbag and run Kimera-VIO.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/RosBagDataProvider.h"

#include <glog/logging.h>

#include <rosgraph_msgs/Clock.h>

#include <kimera-vio/pipeline/Pipeline-definitions.h>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosbagDataProvider::RosbagDataProvider(const VioParams& vio_params)
    : RosDataProviderInterface(vio_params),
      rosbag_data_(),
      rosbag_path_(""),
      left_imgs_topic_(""),
      right_imgs_topic_(""),
      imu_topic_(""),
      gt_odom_topic_(""),
      clock_pub_(),
      imu_pub_(),
      left_img_pub_(),
      right_img_pub_(),
      gt_odometry_pub_(),
      timestamp_last_frame_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_kf_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_imu_(std::numeric_limits<Timestamp>::min()),
      timestamp_last_gt_(std::numeric_limits<Timestamp>::min()),
      k_(0u),
      k_last_kf_(0u),
      k_last_imu_(0u),
      k_last_gt_(0u),
      log_gt_data_(false),
      is_header_written_poses_vio_(false),
      output_gt_poses_csv_("traj_gt.csv") {
  CHECK(nh_private_.getParam("rosbag_path", rosbag_path_));
  CHECK(nh_private_.getParam("left_cam_rosbag_topic", left_imgs_topic_));
  CHECK(nh_private_.getParam("right_cam_rosbag_topic", right_imgs_topic_));
  CHECK(nh_private_.getParam("imu_rosbag_topic", imu_topic_));
  CHECK(nh_private_.getParam("ground_truth_odometry_rosbag_topic",
                             gt_odom_topic_));
  CHECK(nh_private_.getParam("log_gt_data", log_gt_data_));

  LOG(INFO) << "Constructing RosbagDataProvider from path: \n"
            << " - Rosbag Path: " << rosbag_path_.c_str() << '\n'
            << "With ROS topics: \n"
            << " - Left cam: " << left_imgs_topic_.c_str() << '\n'
            << " - Right cam: " << right_imgs_topic_.c_str() << '\n'
            << " - IMU: " << imu_topic_.c_str() << '\n'
            << " - GT odom: " << gt_odom_topic_.c_str();

  CHECK(!rosbag_path_.empty());
  CHECK(!left_imgs_topic_.empty());
  CHECK(!right_imgs_topic_.empty());
  CHECK(!imu_topic_.empty());

  // Ros publishers specific to rosbag data provider
  static constexpr size_t kQueueSize = 10u;

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", kQueueSize);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, kQueueSize);
  left_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(left_imgs_topic_, kQueueSize);
  right_img_pub_ =
      nh_.advertise<sensor_msgs::Image>(right_imgs_topic_, kQueueSize);

  if (!gt_odom_topic_.empty()) {
    gt_odometry_pub_ =
        nh_.advertise<nav_msgs::Odometry>(gt_odom_topic_, kQueueSize);
  }
}

void RosbagDataProvider::initialize() {
  CHECK_EQ(k_, 0u);
  LOG(INFO) << "Initialize Rosbag Data Provider.";
  // Parse data from rosbag first thing:
  CHECK(parseRosbag(rosbag_path_, &rosbag_data_));

  // Autoinitialize if necessary: this changes the values for anyone
  // holding vio_params (such as the VIO), since backend params are a ptr.
  if (vio_params_.backend_params_->autoInitialize_ == 0) {
    vio_params_.backend_params_->initial_ground_truth_state_ =
        getGroundTruthVioNavState(0u);  // Send first gt state.
    LOG(WARNING) << "Using initial ground-truth state for initialization:";
    vio_params_.backend_params_->initial_ground_truth_state_.print();
  }
}

void RosbagDataProvider::sendImuDataToVio() {
  CHECK(imu_single_callback_) << "Did you forget to register the IMU callback?";
  for (const sensor_msgs::ImuConstPtr& imu_msg : rosbag_data_.imu_msgs_) {
    const ImuStamp& imu_data_timestamp = imu_msg->header.stamp.toNSec();
    ImuAccGyr imu_accgyr;
    imu_accgyr(0) = imu_msg->linear_acceleration.x;
    imu_accgyr(1) = imu_msg->linear_acceleration.y;
    imu_accgyr(2) = imu_msg->linear_acceleration.z;
    imu_accgyr(3) = imu_msg->angular_velocity.x;
    imu_accgyr(4) = imu_msg->angular_velocity.y;
    imu_accgyr(5) = imu_msg->angular_velocity.z;
    imu_single_callback_(ImuMeasurement(imu_data_timestamp, imu_accgyr));
  }
}

bool RosbagDataProvider::spin() {
  if (k_ == 0u) {
    // Send IMU data directly to VIO for speed boost
    sendImuDataToVio();
  }

  // We break the while loop (but increase k_!) if we run in sequential mode.
  while (k_ < rosbag_data_.left_imgs_.size()) {
    if (nh_.ok() && ros::ok() && !ros::isShuttingDown() && !shutdown_) {
      const Timestamp& timestamp_frame_k = rosbag_data_.timestamps_.at(k_);

      static const CameraParams& left_cam_info =
          vio_params_.camera_params_.at(0);
      static const CameraParams& right_cam_info =
          vio_params_.camera_params_.at(1);

      if (timestamp_frame_k > timestamp_last_frame_) {
        // Send left frame data to Kimera:
        CHECK(left_frame_callback_)
            << "Did you forget to register the left frame callback?";
        left_frame_callback_(VIO::make_unique<Frame>(
            k_,
            timestamp_frame_k,
            left_cam_info,
            readRosImage(rosbag_data_.left_imgs_.at(k_))));

        // Send right frame data to Kimera:
        if (vio_params_.frontend_type_ == VIO::FrontendType::kStereoImu) {
          CHECK(right_frame_callback_)
              << "Did you forget to register the right frame callback?";
          right_frame_callback_(VIO::make_unique<Frame>(
              k_,
              timestamp_frame_k,
              right_cam_info,
              readRosImage(rosbag_data_.right_imgs_.at(k_))));
        }

        VLOG(10) << "Sent left/right images to VIO for frame k = " << k_;

        // Publish VIO output if any.
        // TODO(Toni) this could go faster if running in another thread or
        // node...

        // // Publish LCD output if any.
        // LcdOutput::Ptr lcd_output = nullptr;
        // if (lcd_output_queue_.pop(lcd_output)) {
        //   publishLcdOutput(lcd_output);
        // }

        timestamp_last_frame_ = timestamp_frame_k;
      } else {
        if (timestamp_frame_k == timestamp_last_frame_) {
          LOG(WARNING)
              << "Timestamps for current and previous frames are equal! \n"
              << "This should not happen, dropping this frame... \n "
              << "- Offending timestamp: " << timestamp_frame_k;
        } else {
          LOG(WARNING) << "Skipping frame: " << k_ << '\n'
                       << " Frame timestamps out of order:\n"
                       << " Timestamp Current Frame: " << timestamp_frame_k
                       << "\n"
                       << " Timestamp Last Frame:    " << timestamp_last_frame_;
        }
      }

      publishRosbagInfo(timestamp_frame_k);
      ros::spinOnce();
    } else {
      LOG(ERROR) << "ROS SHUTDOWN requested, stopping rosbag spin.";
      ros::shutdown();
      return false;
    }

    // Next iteration
    k_++;
    if (!vio_params_.parallel_run_) {
      // Break while loop (but keep increasing k_!) if we run in sequential
      // mode. We actually return instead of break, to avoid re-printing that
      // the rosbag processing has finished.
      return true;
    }
  }  // End of for loop over rosbag images.
  LOG(INFO) << "Rosbag processing finished.";

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
    LOG_IF(WARNING, vio_params_.backend_params_->autoInitialize_ != 0)
        << "Provided a gt_odom_topic; but autoInitialize "
           "(BackendParameters.yaml) is not set to 0,"
           " meaning no ground-truth initialization will be done... "
           "Are you sure you don't want to use gt?)";
    topics.push_back(gt_odom_topic_);
  } else {
    // TODO(Toni): autoinit should be a bool...
    CHECK_EQ(vio_params_.backend_params_->autoInitialize_, 1)
        << "Requested ground-truth initialization, but no gt_odom_topic "
           "was given. Make sure you set ground_truth_odometry_rosbag_topic, "
           "or turn autoInitialize to false in BackendParameters.yaml.";
  }

  // Query rosbag for given topics
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  int imu_msg_count = 0;
  // Keep track of this since we expect IMU data before an image.
  bool start_parsing_stereo = false;
  // For some datasets, we have duplicated measurements for the same time.
  Timestamp last_imu_timestamp = 0;
  for (const rosbag::MessageInstance& msg : view) {
    const std::string& msg_topic = msg.getTopic();

    // Check if msg is an IMU measurement.
    sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
    if (imu_msg != nullptr && msg_topic == imu_topic_) {
      const ImuStamp& imu_data_timestamp = imu_msg->header.stamp.toNSec();
      if (imu_data_timestamp > last_imu_timestamp) {
        VLOG(10) << "IMU msg count: " << imu_msg_count++;
        rosbag_data->imu_msgs_.push_back(imu_msg);
        last_imu_timestamp = imu_data_timestamp;
      } else {
        if (imu_data_timestamp - last_imu_timestamp == 0u) {
          LOG(WARNING) << "IMU timestamps in rosbag are repeated!\n"
                       << "Offending timestamp: " << imu_data_timestamp;
        } else {
          LOG(FATAL) << "IMU timestamps in rosbag are out of order: consider "
                     << "re-ordering rosbag: \n"
                     << "- Current IMU timestamp: " << imu_data_timestamp
                     << '\n'
                     << "- Last IMU timestamp: " << last_imu_timestamp << '\n'
                     << "Difference (current - last) = "
                     << imu_data_timestamp - last_imu_timestamp;
        }
      }
      start_parsing_stereo = true;
      continue;
    }

    // Check if msg is an image.
    sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();
    if (img_msg != nullptr) {
      if (start_parsing_stereo) {
        // Check left or right image.
        if (msg_topic == left_imgs_topic_) {
          // Timestamp is in nanoseconds
          rosbag_data->timestamps_.push_back(img_msg->header.stamp.toNSec());
          rosbag_data->left_imgs_.push_back(img_msg);
        } else if (msg_topic == right_imgs_topic_) {
          rosbag_data->right_imgs_.push_back(img_msg);
        } else {
          LOG(WARNING) << "Img with unexpected topic: " << msg_topic;
        }
      } else {
        LOG(WARNING) << "Skipping first frame in rosbag, since IMU data not "
                        "yet available.";
      }
      continue;
    }

    // Check if msg is a ground-truth odometry message.
    nav_msgs::OdometryConstPtr gt_odom_msg =
        msg.instantiate<nav_msgs::Odometry>();
    if (gt_odom_msg != nullptr) {
      if (msg_topic == gt_odom_topic_) {
        rosbag_data->gt_odometry_.push_back(gt_odom_msg);
        if (log_gt_data_) {
          logGtData(gt_odom_msg);
        }
      } else {
        LOG(ERROR) << "Unrecognized topic name for odometry msg. We were"
                      " expecting ground-truth odometry on this topic: "
                   << msg_topic;
      }
    } else {
      LOG(ERROR) << "Could not find the type of this rosbag msg from topic:\n"
                 << msg_topic;
    }
  }
  bag.close();

  // Sanity check:
  LOG_IF(FATAL,
         rosbag_data->left_imgs_.size() == 0 ||
             rosbag_data->right_imgs_.size() == 0)
      << "No images parsed from rosbag.";
  LOG_IF(FATAL,
         rosbag_data->left_imgs_.size() != rosbag_data->right_imgs_.size())
      << "Unequal number of images from left and right cameras.";
  LOG_IF(FATAL, rosbag_data->imu_msgs_.size() <= rosbag_data->left_imgs_.size())
      << "Less than or equal number of imu data as image data.";
  LOG_IF(FATAL,
         !gt_odom_topic_.empty() && rosbag_data->gt_odometry_.size() == 0)
      << "Requested to parse ground-truth odometry, but parsed 0 msgs.";
  LOG_IF(WARNING,
         !gt_odom_topic_.empty() &&
             rosbag_data->gt_odometry_.size() < rosbag_data->left_imgs_.size())
      << "Fewer ground_truth data than image data.";
  LOG(INFO) << "Finished parsing rosbag data.";
  return true;
}

void RosbagDataProvider::logGtData(const nav_msgs::OdometryConstPtr& odometry) {
  CHECK(odometry);
  // We log the poses in csv format for later alignement and analysis.
  std::ofstream& output_stream = output_gt_poses_csv_.ofstream_;
  bool& is_header_written = is_header_written_poses_vio_;

  // First, write header, but only once.
  if (!is_header_written) {
    output_stream << "#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz,"
                  << "bgx,bgy,bgz,bax,bay,baz" << std::endl;
    is_header_written = true;
  }
  output_stream << odometry->header.stamp.toNSec() << ","  //
                << odometry->pose.pose.position.x << ","   //
                << odometry->pose.pose.position.y << ","   //
                << odometry->pose.pose.position.z << ","   //
                << odometry->pose.pose.orientation.w << ","
                << odometry->pose.pose.orientation.x << ","
                << odometry->pose.pose.orientation.y << ","
                << odometry->pose.pose.orientation.z << ","
                << odometry->twist.twist.linear.x << ","
                << odometry->twist.twist.linear.y << ","
                << odometry->twist.twist.linear.z << "," << 0.0 << ","  //
                << 0.0 << ","                                           //
                << 0.0 << ","                                           //
                << 0.0 << ","                                           //
                << 0.0 << ","                                           //
                << 0.0                                                  //
                << std::endl;
}

VioNavState RosbagDataProvider::getGroundTruthVioNavState(
    const size_t& k_frame) const {
  CHECK_LT(k_frame, rosbag_data_.gt_odometry_.size());
  nav_msgs::Odometry gt_odometry = *(rosbag_data_.gt_odometry_.at(k_frame));
  VioNavState vio_nav_state;
  utils::rosOdometryToVioNavState(gt_odometry, nh_private_, &vio_nav_state);
  return vio_nav_state;
}

void RosbagDataProvider::publishRosbagInfo(const Timestamp& timestamp) {
  publishInputs(timestamp);
  publishClock(timestamp);
}

void RosbagDataProvider::publishClock(const Timestamp& timestamp) const {
  rosgraph_msgs::Clock clock;
  clock.clock.fromNSec(timestamp);
  clock_pub_.publish(clock);
}

void RosbagDataProvider::publishInputs(const Timestamp& timestamp_kf) {
  // Publish all imu messages to ROS:
  if (k_last_imu_ < rosbag_data_.imu_msgs_.size()) {
    while (timestamp_last_imu_ < timestamp_kf &&
           k_last_imu_ < rosbag_data_.imu_msgs_.size()) {
      imu_pub_.publish(rosbag_data_.imu_msgs_.at(k_last_imu_));
      timestamp_last_imu_ =
          rosbag_data_.imu_msgs_.at(k_last_imu_)->header.stamp.toNSec();
      k_last_imu_++;
    }
  }

  // Publish ground-truth data if available:
  if (k_last_gt_ < rosbag_data_.gt_odometry_.size()) {
    while (timestamp_last_gt_ < timestamp_kf &&
           k_last_gt_ < rosbag_data_.gt_odometry_.size()) {
      gt_odometry_pub_.publish(rosbag_data_.gt_odometry_.at(k_last_gt_));
      timestamp_last_gt_ =
          rosbag_data_.gt_odometry_.at(k_last_gt_)->header.stamp.toNSec();
      k_last_gt_++;
    }
  }

  // Publish left and right images:
  if (k_last_kf_ < rosbag_data_.left_imgs_.size() &&
      k_last_kf_ < rosbag_data_.right_imgs_.size()) {
    while (timestamp_last_kf_ < timestamp_kf &&
           k_last_kf_ < rosbag_data_.left_imgs_.size() &&
           k_last_kf_ < rosbag_data_.right_imgs_.size()) {
      left_img_pub_.publish(rosbag_data_.left_imgs_.at(k_last_kf_));
      right_img_pub_.publish(rosbag_data_.right_imgs_.at(k_last_kf_));
      timestamp_last_kf_ = rosbag_data_.timestamps_.at(k_last_kf_);
      k_last_kf_++;
    }
  }
}

}  // namespace VIO
