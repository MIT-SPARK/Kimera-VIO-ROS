/**
 * @file   base-data-source.h
 * @brief  Base class for ROS wrappers for KimeraVIO.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/core/matx.hpp"

#define PCL_NO_PRECOMPILE  // Define this before you include any PCL headers
                           // to include the templated algorithms
#include "pcl/point_types.h"
#include "pcl_msgs/msg/polygon_mesh.hpp"
//#include "pcl_ros/point_cloud.h"
//#include "pcl_ros/point_cloud.h"


#include "image_transport/subscriber_filter.h"
#include "rclcpp/rclcpp.hpp"
//#include "tf/transform_broadcaster.h"

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/ThreadsafeQueue.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl_msgs/msg/polygon_mesh.hpp"
#include "pose_graph_msgs/msg/pose_graph.hpp"
#include "pose_graph_msgs/msg/pose_graph_edge.hpp"
#include "pose_graph_msgs/msg/pose_graph_node.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
//#include "sensor_msgs/msg/image_encodings.h"

//#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace VIO {

/**
 * @breif Struct to hold mesh vertex data.
 */
struct PointNormalUV {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float u;  // Texture coordinates.
  float v;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class RosDataProviderInterface : public DataProviderInterface, public rclcpp::Node{
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDataProviderInterface);
  KIMERA_POINTER_TYPEDEFS(RosDataProviderInterface);

  RosDataProviderInterface(
      const std::string & node_name,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~RosDataProviderInterface();

 public:
  inline void callbackBackendOutput(const VIO::BackendOutput::Ptr& output) {
    backend_output_queue_.push(output);
  }

  inline void callbackFrontendOutput(const VIO::FrontendOutput::Ptr& output) {
    frontend_output_queue_.push(output);
  }

  inline void callbackMesherOutput(const VIO::MesherOutput::Ptr& output) {
    mesher_output_queue_.push(output);
  }

  inline void callbackLcdOutput(const VIO::LcdOutput::Ptr& output) {
    lcd_output_queue_.push(output);
  }

 protected:
  const cv::Mat readRosImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  const cv::Mat readRosDepthImage(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

  // Publish VIO outputs.
  virtual void publishBackendOutput(const BackendOutput::Ptr& output);

  virtual void publishFrontendOutput(const FrontendOutput::Ptr& output) const;

  virtual void publishMesherOutput(const MesherOutput::Ptr& output) const;

  virtual bool publishSyncedOutputs();

  // Publish all outputs for LCD
  // TODO(marcus): make like other outputs
  virtual void publishLcdOutput(const LcdOutput::Ptr& lcd_output);

  // Publish static transforms (for camera frames) to the tf tree
  void publishStaticTf(const gtsam::Pose3& pose,
                       const std::string& parent_frame_id,
                       const std::string& child_frame_id);

 protected:
  // Define image transport for this and derived classes.
  std::unique_ptr<image_transport::ImageTransport> it_;

  // Define frame ids for odometry message
  std::string frame_id_world_;
  std::string frame_id_base_link_;
  std::string frame_id_map_;
  // std::string frame_id_left_cam_;
  // std::string frame_id_right_cam_;

  // Queues to store and retrieve VIO output in a thread-safe way.
  ThreadsafeQueue<BackendOutput::Ptr> backend_output_queue_;
  ThreadsafeQueue<FrontendOutput::Ptr> frontend_output_queue_;
  ThreadsafeQueue<MesherOutput::Ptr> mesher_output_queue_;
  ThreadsafeQueue<LcdOutput::Ptr> lcd_output_queue_;

 private:
  void publishTf(const BackendOutput::Ptr& output);

  void publishTimeHorizonPointCloud(const BackendOutput::Ptr& output) const;

  void publishPerFrameMesh3D(const MesherOutput::Ptr& output) const;

  void publishTimeHorizonMesh3D(const MesherOutput::Ptr& output) const;

  void publishState(const BackendOutput::Ptr& output) const;

  void publishFrontendStats(const FrontendOutput::Ptr& output) const;

  void publishResiliency(const FrontendOutput::Ptr& frontend_output,
                         const BackendOutput::Ptr& backend_output) const;

  void publishImuBias(const BackendOutput::Ptr& output) const;

  // Publish LCD/PGO outputs.
  void publishTf(const LcdOutput::Ptr& lcd_output);

  void publishOptimizedTrajectory(const LcdOutput::Ptr& lcd_output) const;

  void publishPoseGraph(const LcdOutput::Ptr& lcd_output);

  void updateNodesAndEdges(const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values);

  void updateRejectedEdges();

  pose_graph_msgs::msg::PoseGraph getPosegraphMsg();

  // Publish/print debugging information.
  void publishDebugImage(const Timestamp& timestamp,
                         const cv::Mat& debug_image) const;

  void printParsedParams() const;

 private:
  // Define publisher for debug images.
  image_transport::Publisher debug_img_pub_;

  // Typedefs
//  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

  // Publishers
//  rclcpp::Publisher<PointCloudXYZRGB> pointcloud_pub_;
  // Published 3d mesh per frame (not time horizon of opitimization!)
  rclcpp::Publisher<pcl_msgs::msg::PolygonMesh>::SharedPtr mesh_3d_frame_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr resiliency_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frontend_stats_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_bias_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<pose_graph_msgs::msg::PoseGraph>::SharedPtr posegraph_pub_;

  // Define tf broadcaster for world to base_link (IMU) and to map (PGO).
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Stored pose graph related objects
  std::vector<pose_graph_msgs::msg::PoseGraphEdge> loop_closure_edges_;
  std::vector<pose_graph_msgs::msg::PoseGraphEdge> odometry_edges_;
  std::vector<pose_graph_msgs::msg::PoseGraphEdge> inlier_edges_;
  std::vector<pose_graph_msgs::msg::PoseGraphNode> pose_graph_nodes_;
};

}  // namespace VIO

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VIO::PointNormalUV,
    (float, x, x)(float, y, y)(float, x, z)(float, normal_x, normal_x)(
        float,
        normal_y,
        normal_y)(float, normal_z, normal_z)(float, u, u)(float, v, v))
