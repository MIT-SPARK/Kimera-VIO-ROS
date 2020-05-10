#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#define PCL_NO_PRECOMPILE  // Define this before you include any PCL headers
                           // to include the templated algorithms
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>

#include <kimera-vio/pipeline/QueueSynchronizer.h>
#include <kimera-vio/visualizer/Display.h>
#include <kimera-vio/visualizer/Visualizer3D.h>

#include <kimera_vio_ros/RosPublishers.h>

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

class RosDisplay : public DisplayBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RosDisplay);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosDisplay);

 public:
  RosDisplay(const VioParams& vio_params)
      : DisplayBase(),
        nh_(),
        nh_private_("~"),
        vio_params_(vio_params),
        backend_output_queue_("Backend output ROS"),
        frame_rate_frontend_output_queue_("Frame Rate Frontend output ROS"),
        keyframe_rate_frontend_output_queue_(
            "Keyframe Rate Frontend output ROS"),
        mesher_output_queue_("Mesher output ROS"),
        lcd_output_queue_("LCD output ROS"),
        image_publishers_(nullptr) {
    image_publishers_ = VIO::make_unique<ImagePublishers>(nh_private_);

    // Get ROS params
    CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
    CHECK(!base_link_frame_id_.empty());
    CHECK(nh_private_.getParam("world_frame_id", world_frame_id_));
    CHECK(!world_frame_id_.empty());
    CHECK(nh_private_.getParam("map_frame_id", map_frame_id_));
    CHECK(!map_frame_id_.empty());
    CHECK(nh_private_.getParam("left_cam_frame_id", left_cam_frame_id_));
    CHECK(!left_cam_frame_id_.empty());
    CHECK(nh_private_.getParam("right_cam_frame_id", right_cam_frame_id_));
    CHECK(!right_cam_frame_id_.empty());

    // Publishers
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1, true);
    frontend_stats_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("frontend_stats", 1);
    resiliency_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("resiliency", 1);
    imu_bias_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("imu_bias", 1);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("optimized_trajectory", 1);
    posegraph_pub_ =
        nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1);
    pointcloud_pub_ =
        nh_.advertise<PointCloudXYZRGB>("time_horizon_pointcloud", 1, true);
    mesh_3d_frame_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("mesh", 1, true);

    publishStaticTf(vio_params_.camera_params_.at(0).body_Pose_cam_,
                    base_link_frame_id_,
                    left_cam_frame_id_);
    publishStaticTf(vio_params_.camera_params_.at(1).body_Pose_cam_,
                    base_link_frame_id_,
                    right_cam_frame_id_);
  }
  virtual ~RosDisplay() = default;

 public:
  // Spins the display once to render the visualizer output.
  virtual void spinOnce(VisualizerOutput::UniquePtr&& viz_output) {
    CHECK(viz_output);
    // Display 2D images.
    spin2dWindow(*viz_output);
    // ros::spinOnce();
  }

 public:
  inline void callbackBackendOutput(const VIO::BackendOutput::Ptr& output) {
    backend_output_queue_.push(output);
  }

  inline void callbackFrontendOutput(const VIO::FrontendOutput::Ptr& output) {
    // TODO(Toni): pushing twice to two different queues bcs we are missing
    // the functionality to ".front()" a queue, now we can just pop()...
    // Perhaps we should make all our threadsafe queues temporally aware
    // (meaning you can query the time of the message directly)...
    frame_rate_frontend_output_queue_.push(output);
    if (output && output->is_keyframe_) {
      keyframe_rate_frontend_output_queue_.push(output);
    }
  }

  inline void callbackMesherOutput(const VIO::MesherOutput::Ptr& output) {
    mesher_output_queue_.push(output);
  }

  inline void callbackLcdOutput(const VIO::LcdOutput::Ptr& output) {
    lcd_output_queue_.push(output);
  }

  virtual void shutdownQueues() {
    backend_output_queue_.shutdown();
    frame_rate_frontend_output_queue_.shutdown();
    keyframe_rate_frontend_output_queue_.shutdown();
    mesher_output_queue_.shutdown();
    lcd_output_queue_.shutdown();
  }

 protected:
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

  pose_graph_tools::PoseGraph getPosegraphMsg();

  void publishDebugImage(const Timestamp& timestamp,
                         const cv::Mat& debug_image) const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Just to get left/right cam to body tfs...
  VioParams vio_params_;

  // Define image publishers manager
  std::unique_ptr<ImagePublishers> image_publishers_;

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

 private:
  // Define frame ids for odometry message
  std::string world_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;
  std::string left_cam_frame_id_;
  std::string right_cam_frame_id_;

  // Queues to store and retrieve VIO output in a thread-safe way.
  ThreadsafeQueue<BackendOutput::Ptr> backend_output_queue_;
  ThreadsafeQueue<FrontendOutput::Ptr> frame_rate_frontend_output_queue_;
  ThreadsafeQueue<FrontendOutput::Ptr> keyframe_rate_frontend_output_queue_;
  ThreadsafeQueue<MesherOutput::Ptr> mesher_output_queue_;
  ThreadsafeQueue<LcdOutput::Ptr> lcd_output_queue_;

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
