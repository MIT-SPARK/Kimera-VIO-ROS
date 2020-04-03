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
    RosDataProviderInterfaceExposed(const VioParams& vio_params) 
      : RosDataProviderInterface(vio_params) {} 
    using RosDataProviderInterface::base_link_frame_id_;
    using RosDataProviderInterface::world_frame_id_;
    using RosDataProviderInterface::map_frame_id_;
    using RosDataProviderInterface::left_cam_frame_id_;
    using RosDataProviderInterface::right_cam_frame_id_;
    static const size_t kTfLookupTimeout = 
                  RosDataProviderInterface::kTfLookupTimeout;
};


/* ************************************************************************** */

class TestRosDataProviderInterface : public ::testing::Test {
 public:
  TestRosDataProviderInterface() :
        publishing_node_(),
        parameter_server_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_)
        {
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
      publishing_node_.subscribe<std_msgs::Float64MultiArray>("frontend_stats", 1, nullptr);
  ros::Subscriber resiliency_sub = 
      publishing_node_.subscribe<std_msgs::Float64MultiArray>("resiliency", 1, nullptr);
  ros::Subscriber imu_bias_sub = 
      publishing_node_.subscribe<std_msgs::Float64MultiArray>("imu_bias", 1, nullptr);
  ros::Subscriber trajectory_sub = 
      publishing_node_.subscribe<nav_msgs::Path>("optimized_trajectory", 1, nullptr);
  ros::Subscriber posegraph_sub = 
      publishing_node_.subscribe<pose_graph_tools::PoseGraph>("pose_graph", 1, nullptr);
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

}  // namespace VIO
