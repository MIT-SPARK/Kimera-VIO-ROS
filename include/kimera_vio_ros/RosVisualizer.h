/**
 * @file   RosVisualizer.h
 * @brief  Equivalent Kimera Visualizer but in ROS. Publishes 3D data to ROS.
 * @author Antoni Rosinol
 */

#pragma once

#include <string>
#include <vector>

#define PCL_NO_PRECOMPILE  // Define this before you include any PCL headers
                           // to include the templated algorithms
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>

#include <glog/logging.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>

#include <kimera-vio/backend/VioBackend-definitions.h>
#include <kimera-vio/frontend/FrontendOutputPacketBase.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/mesh/Mesher-definitions.h>
#include <kimera-vio/visualizer/Visualizer3D.h>

#include "kimera_vio_ros/RosLoopClosureVisualizer.h"
#include "kimera_vio_ros/RosPublishers.h"

namespace VIO {

/**
 * @brief The PointNormalUV struct holds mesh vertex data.
 */
struct PointNormalUV {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float u;  // Texture coordinates.
  float v;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class RosVisualizer : public Visualizer3D {
 public:
  KIMERA_POINTER_TYPEDEFS(RosVisualizer);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosVisualizer);

 public:
  RosVisualizer(const VioParams& vio_params);
  virtual ~RosVisualizer() = default;

 public:
  /**
   * @brief spinOnce
   * Spins the display once to render the visualizer output.
   * @param viz_input
   */
  VisualizerOutput::UniquePtr spinOnce(
      const VisualizerInput& viz_input) override;

 protected:
  // Publish VIO outputs.
  virtual void publishBackendOutput(const BackendOutput::ConstPtr& output);

  virtual void publishFrontendOutput(const FrontendOutputPacketBase::ConstPtr& output) const;

  virtual void publishMesherOutput(const MesherOutput::ConstPtr& output) const;

 private:
  void publishTimeHorizonPointCloud(
      const BackendOutput::ConstPtr& output) const;

  void publishPerFrameMesh3D(const MesherOutput::ConstPtr& output) const;

  // void publishTimeHorizonMesh3D(const MesherOutput::ConstPtr& output) const;

  void publishState(const BackendOutput::ConstPtr& output) const;

  void publishFrontendStats(const FrontendOutputPacketBase::ConstPtr& output) const;

  void publishResiliency(const FrontendOutputPacketBase::ConstPtr& frontend_output,
                         const BackendOutput::ConstPtr& backend_output) const;

  void publishImuBias(const BackendOutput::ConstPtr& output) const;

  void publishTf(const BackendOutput::ConstPtr& output);

  void publishDebugImage(const Timestamp& timestamp,
                         const cv::Mat& debug_image) const;

 private:
  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS publishers
  ros::Publisher pointcloud_pub_;
  //! Published 3d mesh per frame (not time horizon of opitimization!)
  ros::Publisher mesh_3d_frame_pub_;
  ros::Publisher odometry_pub_;
  ros::Publisher resiliency_pub_;
  ros::Publisher frontend_stats_pub_;
  ros::Publisher imu_bias_pub_;

  //! Define tf broadcaster for world to base_link (IMU) and to map (PGO).
  tf::TransformBroadcaster tf_broadcaster_;

 private:
  //! Define frame ids for odometry message
  std::string world_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;

  cv::Size image_size_;

  //! Define image publishers manager
  std::unique_ptr<ImagePublishers> image_publishers_;

  RosLoopClosureVisualizer lcd_visualizer_;

 private:
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
