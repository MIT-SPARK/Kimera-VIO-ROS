/**
 * @file   testRosDataProviderInterface.cpp
 * @brief  Unit tests for base class for ROS wrappers for Kimera-VIO.
 * @author Andrew Violette
 */

#include <memory>
#include <string>

#include <glog/logging.h>
#include <gtest/gtest.h>

// Data types for ros subscriptions
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

// Includes for ROS interfacing
#include <tf2_ros/transform_listener.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"
// Needed to mock up StereoFrames, needed for FrontendOutput
DECLARE_bool(images_rectified);

namespace VIO {

// Path for default VioParams parsing
// Empty because default values will serve our purpose
static constexpr char params_folder_path[] = "";

// Constants for ROS TF
static constexpr char dummy_base_link_frame_id_[] = "dummy_base_link_frame_id";
static constexpr char dummy_world_frame_id_[] = "dummy_world_frame_id";
static constexpr char dummy_map_frame_id_[] = "dummy_map_frame_id";
static constexpr char dummy_left_cam_frame_id_[] = "dummy_left_cam_frame_id";
static constexpr char dummy_right_cam_frame_id_[] = "dummy_right_cam_frame_id";

// Expose protected members for testing
// see: StackOverflow's "How do I unit test a protected method in C++?" KrisTC
class RosDataProviderInterfaceExposed : public RosDataProviderInterface {
 public:
  explicit RosDataProviderInterfaceExposed(const VioParams& vio_params)
      : RosDataProviderInterface(vio_params) {}
  using RosDataProviderInterface::base_link_frame_id_;
  using RosDataProviderInterface::left_cam_frame_id_;
  using RosDataProviderInterface::map_frame_id_;
  using RosDataProviderInterface::right_cam_frame_id_;
  using RosDataProviderInterface::world_frame_id_;
  static const size_t kTfLookupTimeout =
      RosDataProviderInterface::kTfLookupTimeout;

  using RosDataProviderInterface::backend_output_queue_;
  using RosDataProviderInterface::keyframe_rate_frontend_output_queue_;
  using RosDataProviderInterface::mesher_output_queue_;

  using RosDataProviderInterface::publishSyncedOutputs;
  // To isolate current testing method, stub the methods it calls
  bool mock_publish_backend_output_ = false;
  size_t mock_publish_backend_call_count_ = 0;
  void publishBackendOutput(const BackendOutput::Ptr& output) {
    if (!mock_publish_backend_output_) {
      RosDataProviderInterface::publishBackendOutput(output);
    } else {
      mock_publish_backend_call_count_++;
    }
      }
      bool mock_publish_mesher_output_ = false;
      // publishMesherOutput is const; get around it with a pointer
      std::shared_ptr<size_t> mock_publish_mesher_call_count_ = nullptr;
      void publishMesherOutput(const MesherOutput::Ptr& output) const {
        if (!mock_publish_mesher_output_) {
          RosDataProviderInterface::publishMesherOutput(output);
        } else {
          CHECK(mock_publish_mesher_call_count_)
              << "You must set the mock_publish_mesher_call_count_ pointer!";
          (*mock_publish_mesher_call_count_)++;
        }
      }
};

class RosDataProviderInterfacePrivateStubbed
    : public RosDataProviderInterfaceExposed {
 public:
  size_t mock_publish_resiliency_call_count_ = 0;
  void publishResiliency(const FrontendOutput::Ptr& frontend_output,
                         const BackendOutput::Ptr& backend_output) {
    mock_publish_resiliency_call_count_++;
  }
};


/* ************************************************************************** */

class TestRosDataProviderInterface : public ::testing::Test {
 public:
  TestRosDataProviderInterface()
      : publishing_node_(),
        parameter_server_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_) {
    setFrameIDs();
    initializeVioParams();
  }

  ~TestRosDataProviderInterface() { }

 protected:
  ros::NodeHandle publishing_node_;
  ros::NodeHandle parameter_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  VIO::VioParams::Ptr dummy_vio_params_;

  void setFrameIDs() {
    parameter_server_.setParam("base_link_frame_id", dummy_base_link_frame_id_);
    parameter_server_.setParam("world_frame_id", dummy_world_frame_id_);
    parameter_server_.setParam("map_frame_id", dummy_map_frame_id_);
    parameter_server_.setParam("left_cam_frame_id", dummy_left_cam_frame_id_);
    parameter_server_.setParam("right_cam_frame_id", dummy_right_cam_frame_id_);
  }

  void initializeVioParams() {
    dummy_vio_params_ =
        std::make_shared<VIO::VioParams>(std::string(params_folder_path));
    // need to push two dummy camera params-- assumes a left and right camera
    VIO::CameraParams dummy_cam_params;
    dummy_vio_params_->camera_params_.push_back(dummy_cam_params);
    dummy_vio_params_->camera_params_.push_back(dummy_cam_params);
  }

  FrontendOutput::Ptr makeDummyFrontendOutput(const Timestamp& timestamp) {
    // There is currently no clean way to satisfy the dependency injection
    // Make a dummy input for each of the FrontEndOutput args
    VIO::StatusStereoMeasurementsPtr dummy_stereo_measurements =
        std::make_shared<VIO::StatusStereoMeasurements>();
    gtsam::Pose3 dummy_pose;
    VIO::StereoFrame::Ptr dummy_stereo_frame = makeDummyStereoFrame(timestamp);
    VIO::ImuFrontEnd::PimPtr dummy_pim_ptr;
    cv::Mat dummy_feature_tracks;
    VIO::DebugTrackerInfo dummy_debug_tracking;
    VIO::FrontendOutput::Ptr dummy_frontend =
        std::make_shared<VIO::FrontendOutput>(true,
                                              dummy_stereo_measurements,
                                              TrackingStatus::INVALID,
                                              dummy_pose,
                                              *dummy_stereo_frame,
                                              dummy_pim_ptr,
                                              dummy_feature_tracks,
                                              dummy_debug_tracking);

    return dummy_frontend;
  }

  VIO::StereoFrame::Ptr makeDummyStereoFrame(const Timestamp& timestamp) {
    // StereoFrame has nested dependencies and some constructor issues
    VIO::CameraParams dummy_params;
    dummy_params.R_rectify_ = cv::Mat::zeros(3, 3, CV_32F);
    dummy_params.P_ = cv::Mat::zeros(3, 3, CV_32F);
    VIO::Frame dummy_left_frame(0, timestamp, dummy_params, cv::Mat());
    VIO::Frame dummy_right_frame(0, timestamp, dummy_params, cv::Mat());
    VIO::StereoMatchingParams dummy_stereo_params;
    // Rectification on dummy parameters will result in bad calculations
    // Tell it the images are already rectified
    FLAGS_images_rectified = true;
    VIO::StereoFrame::Ptr dummy_stereo_frame =
        std::make_shared<VIO::StereoFrame>(0,
                                           timestamp,
                                           dummy_left_frame,
                                           dummy_right_frame,
                                           dummy_stereo_params);
    FLAGS_images_rectified = false;

    return dummy_stereo_frame;
  }

  VIO::BackendOutput::Ptr makeDummyBackendOutput(const Timestamp& timestamp) {
    // There is currently no clean way to satisfy the dependency injection
    // Make a dummy input for each of the BackEndOutput args
    gtsam::Values dummy_state;
    gtsam::Pose3 dummy_W_Pose_B;
    VIO::Vector3 dummy_W_Vel_B;
    VIO::ImuBias dummy_imu_bias;
    gtsam::Matrix dummy_state_covariance;
    VIO::FrameId dummy_id;
    int dummy_landmark_count;
    VIO::DebugVioInfo dummy_debug_info;
    VIO::PointsWithIdMap dummy_landmarks_with_id_map;
    VIO::LmkIdToLmkTypeMap dummy_lmk_id_to_lmk_type_map;
    VIO::BackendOutput::Ptr dummy_backend =
        std::make_shared<VIO::BackendOutput>(timestamp,
                                             dummy_state,
                                             dummy_W_Pose_B,
                                             dummy_W_Vel_B,
                                             dummy_imu_bias,
                                             dummy_state_covariance,
                                             dummy_id,
                                             dummy_landmark_count,
                                             dummy_debug_info,
                                             dummy_landmarks_with_id_map,
                                             dummy_lmk_id_to_lmk_type_map);

    return dummy_backend;
  }
};

/* ************************************************************************* */

TEST_F(TestRosDataProviderInterface, constructorTest) {
  RosDataProviderInterfaceExposed test_interface(*dummy_vio_params_);

  // Check to see if we parsed the frame ID's properly
  EXPECT_EQ(dummy_base_link_frame_id_, test_interface.base_link_frame_id_);
  EXPECT_EQ(dummy_world_frame_id_, test_interface.world_frame_id_);
  EXPECT_EQ(dummy_map_frame_id_, test_interface.map_frame_id_);
  EXPECT_EQ(dummy_left_cam_frame_id_, test_interface.left_cam_frame_id_);
  EXPECT_EQ(dummy_right_cam_frame_id_, test_interface.right_cam_frame_id_);

  // Check to see if we advertised the topics that we expected to advertise
  ros::Subscriber odometry_sub =
      publishing_node_.subscribe<nav_msgs::Odometry>("odometry", 1, nullptr);
  ros::Subscriber frontend_stats_sub =
      publishing_node_.subscribe<std_msgs::Float64MultiArray>(
          "frontend_stats", 1, nullptr);
  ros::Subscriber resiliency_sub =
      publishing_node_.subscribe<std_msgs::Float64MultiArray>(
          "resiliency", 1, nullptr);
  ros::Subscriber imu_bias_sub =
      publishing_node_.subscribe<std_msgs::Float64MultiArray>(
          "imu_bias", 1, nullptr);
  ros::Subscriber trajectory_sub = publishing_node_.subscribe<nav_msgs::Path>(
      "optimized_trajectory", 1, nullptr);
  ros::Subscriber posegraph_sub =
      publishing_node_.subscribe<pose_graph_tools::PoseGraph>(
          "pose_graph", 1, nullptr);
  ros::Subscriber pointcloud_sub =
      publishing_node_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>(
          "time_horizon_pointcloud", 1, nullptr);
  ros::Subscriber mesh_3d_frame_sub =
      publishing_node_.subscribe<pcl_msgs::PolygonMesh>("mesh", 1, nullptr);

  EXPECT_EQ(1, odometry_sub.getNumPublishers());
  EXPECT_EQ(1, frontend_stats_sub.getNumPublishers());
  EXPECT_EQ(1, resiliency_sub.getNumPublishers());
  EXPECT_EQ(1, imu_bias_sub.getNumPublishers());
  EXPECT_EQ(1, trajectory_sub.getNumPublishers());
  EXPECT_EQ(1, posegraph_sub.getNumPublishers());
  EXPECT_EQ(1, pointcloud_sub.getNumPublishers());
  EXPECT_EQ(1, mesh_3d_frame_sub.getNumPublishers());

  // Make sure it published the static transforms
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
        dummy_base_link_frame_id_, dummy_left_cam_frame_id_, ros::Time(0),
        ros::Duration(RosDataProviderInterfaceExposed::kTfLookupTimeout));
    EXPECT_EQ(dummy_base_link_frame_id_, transform_stamped.header.frame_id);
    EXPECT_EQ(dummy_left_cam_frame_id_, transform_stamped.child_frame_id);
    EXPECT_EQ(ros::Time(0), transform_stamped.header.stamp);
    // we sent an origin pose, should be all 0's except w
    EXPECT_EQ(1, transform_stamped.transform.rotation.w);
  }
  catch (tf2::TransformException &ex) {
    FAIL() << "Left cam transform failed: " << ex.what();
  }
  try {
    transform_stamped = tf_buffer_.lookupTransform(
        dummy_base_link_frame_id_, dummy_right_cam_frame_id_, ros::Time(0),
        ros::Duration(RosDataProviderInterfaceExposed::kTfLookupTimeout));
    EXPECT_EQ(dummy_base_link_frame_id_, transform_stamped.header.frame_id);
    EXPECT_EQ(dummy_right_cam_frame_id_, transform_stamped.child_frame_id);
    EXPECT_EQ(ros::Time(0), transform_stamped.header.stamp);
    // we sent an origin pose, should be all 0's except w
    EXPECT_EQ(1, transform_stamped.transform.rotation.w);
  }
  catch (tf2::TransformException &ex) {
    FAIL() << "Right cam transform failed: " << ex.what();
  }
}

/* ************************************************************************* */

TEST_F(TestRosDataProviderInterface, synchronizeOutputEmptyTest) {
  RosDataProviderInterfaceExposed test_interface(*dummy_vio_params_);
  test_interface.mock_publish_backend_output_ = true;

  EXPECT_FALSE(test_interface.publishSyncedOutputs());
  EXPECT_EQ(0, test_interface.mock_publish_backend_call_count_);
}

/* ************************************************************************* */

TEST_F(TestRosDataProviderInterface, synchronizeOutputFullTest) {
  RosDataProviderInterfaceExposed test_interface(*dummy_vio_params_);

  Timestamp common_stamp = 10;
  VIO::FrontendOutput::Ptr dummy_frontend_output
      = makeDummyFrontendOutput(common_stamp);
  test_interface.keyframe_rate_frontend_output_queue_
      .push(dummy_frontend_output);

  VIO::BackendOutput::Ptr dummy_backend_output
      = makeDummyBackendOutput(common_stamp);
  test_interface.backend_output_queue_.push(dummy_backend_output);

  VIO::MesherOutput::Ptr dummy_mesher_output
      = std::make_shared<VIO::MesherOutput>(common_stamp);
  test_interface.mesher_output_queue_.push(dummy_mesher_output);

  test_interface.mock_publish_backend_output_ = true;
  test_interface.mock_publish_mesher_output_ = true;
  // Mesher publish call is const, use a pointer as a workaround
  test_interface.mock_publish_mesher_call_count_ =
      std::make_shared<size_t>(0);
  EXPECT_TRUE(test_interface.publishSyncedOutputs());
  EXPECT_EQ(1, test_interface.mock_publish_backend_call_count_);
  EXPECT_EQ(1, *test_interface.mock_publish_mesher_call_count_);
}

}  // namespace VIO
