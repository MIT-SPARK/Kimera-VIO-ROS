/**
 * @file   base-data-source.h
 * @brief  Base class for ROS wrappers for KimeraVIO.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>

#define PCL_NO_PRECOMPILE  // Define this before you include any PCL headers
                           // to include the templated algorithms
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <image_transport/subscriber_filter.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <kimera-vio/common/vio_types.h>
#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/frontend/StereoFrame.h>
#include <kimera-vio/frontend/StereoImuSyncPacket.h>
#include <kimera-vio/frontend/StereoMatchingParams.h>
#include <kimera-vio/mesh/Mesher-definitions.h>
#include <kimera-vio/frontend/VioFrontEndParams.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/utils/ThreadsafeQueue.h>

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

class RosDataProviderInterface : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDataProviderInterface);
  KIMERA_POINTER_TYPEDEFS(RosDataProviderInterface);

  RosDataProviderInterface();

  virtual ~RosDataProviderInterface();

 public:
  inline void callbackBackendOutput(const VIO::BackendOutput::Ptr& output) {
    ROS_INFO("Received Backend Output from pipeline.");
    backend_output_queue_.push(output);
  }

  inline void callbackFrontendOutput(const VIO::FrontendOutput::Ptr& output) {
    ROS_INFO("Received Frontend Output from pipeline.");
    frontend_output_queue_.push(output);
  }

  inline void callbackMesherOutput(const VIO::MesherOutput::Ptr& output) {
    ROS_INFO("Received Mesher Output from pipeline.");
    mesher_output_queue_.push(output);
  }

  inline void callbackLcdOutput(const VIO::LcdOutput::Ptr& output) {
    ROS_INFO("Received Lcd Output from pipeline.");
    lcd_output_queue_.push(output);
  }

 protected:
  cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg) const;

  cv::Mat readRosDepthImage(const sensor_msgs::ImageConstPtr& img_msg) const;

  // Pop and synchronize output packets from queues
  bool getVioOutput(FrontendOutput::Ptr frontend_output,
                    BackendOutput::Ptr backend_output,
                    MesherOutput::Ptr mesher_output,
                    int max_iterations = 10);

  // Publish all outputs by calling individual functions below
  void publishVioOutput(const FrontendOutput::Ptr& frontend_output,
                        const BackendOutput::Ptr& backend_output,
                        const MesherOutput::Ptr& mesher_output);

  // Publish all outputs for LCD
  void publishLcdOutput(const LcdOutput::Ptr& lcd_output);

  // Publish static transforms (for camera frames) to the tf tree
  void publishStaticTf(const gtsam::Pose3& pose,
                       const std::string& parent_frame_id,
                       const std::string& child_frame_id);

 protected:
  // Define Node Handler for general use (Parameter server)
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Define image transport for this and derived classes.
  std::unique_ptr<image_transport::ImageTransport> it_;

  // Define frame ids for odometry message
  std::string world_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;
  std::string left_cam_frame_id_;
  std::string right_cam_frame_id_;

  // Queues to store and retrieve VIO output in a thread-safe way.
  ThreadsafeQueue<BackendOutput::Ptr> backend_output_queue_;
  ThreadsafeQueue<FrontendOutput::Ptr> frontend_output_queue_;
  ThreadsafeQueue<MesherOutput::Ptr> mesher_output_queue_;
  ThreadsafeQueue<LcdOutput::Ptr> lcd_output_queue_;

  // Store IMU data from last frame
  ImuData imu_data_;

 private:
  // Publish VIO outputs.
  void publishTf(const BackendOutput::Ptr& output);
  void publishTimeHorizonPointCloud(
      const Timestamp& timestamp,
      const PointsWithIdMap& points_with_id,
      const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map) const;
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
  pose_graph_tools::PoseGraph getPosegraphMsg();

  // Publish/print debugging information.
  void publishDebugImage(const Timestamp& timestamp,
                         const cv::Mat& debug_image) const;
  void printParsedParams() const;

 private:
  // Define publisher for debug images.
  image_transport::Publisher debug_img_pub_;

  // Publishers
  ros::Publisher pointcloud_pub_;
  // Published 3d mesh per frame (not time horizon of opitimization!)
  ros::Publisher mesh_3d_frame_pub_;
  ros::Publisher odometry_pub_;
  ros::Publisher resiliency_pub_;
  ros::Publisher frontend_stats_pub_;
  ros::Publisher imu_bias_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher posegraph_pub_;

  // Define tf broadcaster for world to base_link (IMU) and to map (PGO).
  tf::TransformBroadcaster tf_broadcaster_;

  // Stored pose graph related objects
  std::vector<pose_graph_tools::PoseGraphEdge> loop_closure_edges_;
  std::vector<pose_graph_tools::PoseGraphEdge> odometry_edges_;
  std::vector<pose_graph_tools::PoseGraphEdge> inlier_edges_;
  std::vector<pose_graph_tools::PoseGraphNode> pose_graph_nodes_;

  // Typedefs
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
};

}  // namespace VIO

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VIO::PointNormalUV,
    (float, x, x)(float, y, y)(float, x, z)(float, normal_x, normal_x)(
        float,
        normal_y,
        normal_y)(float, normal_z, normal_z)(float, u, u)(float, v, v))
