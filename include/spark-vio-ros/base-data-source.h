/**
 * @file   base-data-source.h
 * @brief  Base class for ROS wrappers for spark vio.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include <string>

#define PCL_NO_PRECOMPILE  // Define this before you include any PCL headers
                           // to include the templated algorithms
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <image_transport/subscriber_filter.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>

// TODO(Toni): do we really need all these includes??
// I doubt we are using the imu frontend and the pipeline!
#include <StereoFrame.h>
#include <StereoImuSyncPacket.h>
#include <loopclosure/LoopClosureDetector-definitions.h>
#include <VioFrontEndParams.h>
#include <common/vio_types.h>
#include <datasource/DataSource.h>
#include <utils/ThreadsafeQueue.h>

#include "spark-vio-ros/stereo-image-buffer.h"

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

class RosBaseDataProvider : public DataProvider {
 public:
  RosBaseDataProvider();
  virtual ~RosBaseDataProvider();

  // VIO output callback at keyframe rate.
  void callbackKeyframeRateVioOutput(const SpinOutputPacket& vio_output);

  // LCD/PGO output callback.
  void callbackLoopClosureOutput(
      const LoopClosureDetectorOutputPayload&lcd_output);

 protected:
  // Stereo info
  struct StereoCalibration {
    CameraParams left_camera_info_;
    CameraParams right_camera_info_;
    gtsam::Pose3 camL_Pose_camR_;  // relative pose between cameras
  };

 protected:
  cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg) const;

  cv::Mat readRosDepthImage(const sensor_msgs::ImageConstPtr& img_msg) const;

  // Parse camera calibration info (from param server)
  bool parseCameraData(StereoCalibration* stereo_calib);

  // Parse IMU calibration info (for ros online)
  bool parseImuData(ImuData* imudata, ImuParams* imuparams);

  // Publish all outputs by calling individual functions below
  void publishOutput(const SpinOutputPacket& vio_output);

  // Publish all outputs for LCD
  void publishLCDOutput(const LoopClosureDetectorOutputPayload& lcd_output);

 protected:
  VioFrontEndParams frontend_params_;
  StereoCalibration stereo_calib_;
  SpinOutputPacket vio_output_;

  // Define Node Handler for general use (Parameter server)
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Define image transport for this and derived classes.
  std::unique_ptr<image_transport::ImageTransport> it_;

  // Define frame ids for odometry message
  std::string world_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;

  // Queue to store and retrieve VIO output in a thread-safe way.
  ThreadsafeQueue<SpinOutputPacket> vio_output_queue_;
  ThreadsafeQueue<LoopClosureDetectorOutputPayload> lcd_output_queue_;

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

  // Stored pose graph related objects
  std::vector<pose_graph_tools::PoseGraphEdge> loop_closure_edges_;
  std::vector<pose_graph_tools::PoseGraphEdge> odometry_edges_;
  std::vector<pose_graph_tools::PoseGraphEdge> inlier_edges_;
  std::vector<pose_graph_tools::PoseGraphNode> pose_graph_nodes_;

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

  void publishTimeHorizonPointCloud(
      const Timestamp& timestamp, const PointsWithIdMap& points_with_id,
      const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map) const;
  void publishPerFrameMesh3D(const SpinOutputPacket& vio_output) const;
  void publishTimeHorizonMesh3D(const SpinOutputPacket& vio_output) const;
  void publishState(const SpinOutputPacket& vio_output) const;
  void publishTf(const SpinOutputPacket& vio_output);
  void publishFrontendStats(const SpinOutputPacket& vio_output) const;
  // Publish resiliency statistics
  void publishResiliency(const SpinOutputPacket& vio_output) const;
  void publishImuBias(const SpinOutputPacket& vio_output) const;
  void publishOptimizedTrajectory(
      const LoopClosureDetectorOutputPayload& lcd_output) const;

  void publishPoseGraph(
      const LoopClosureDetectorOutputPayload& lcd_output);

  void UpdateNodesAndEdges(
      const gtsam::NonlinearFactorGraph& nfg,
      const gtsam::Values& values);

  void UpdateRejectedEdges();

  pose_graph_tools::PoseGraph GetPosegraphMsg();

  void publishTf(const LoopClosureDetectorOutputPayload& lcd_output);

  void publishDebugImage(const Timestamp& timestamp,
                         const cv::Mat& debug_image) const;

  // Define tf broadcaster for world to base_link (IMU).
  tf::TransformBroadcaster tf_broadcaster_;
};

}  // namespace VIO

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VIO::PointNormalUV,
    (float, x, x)(float, y, y)(float, x, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(float, u,
                                                              u)(float, v, v))
