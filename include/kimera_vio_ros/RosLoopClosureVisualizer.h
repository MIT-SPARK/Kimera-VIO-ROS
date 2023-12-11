/**
 * @file   RosLoopClosureVisualizer.h
 * @brief  Publishes Loop closure and pose graph data to ROS.
 * @author Yun Chang
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

#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_msgs/PoseGraphEdge.h>
#include <pose_graph_tools_msgs/PoseGraphNode.h>
#include <pose_graph_tools_msgs/VLCFrameQuery.h>
#include <pose_graph_tools_msgs/VLCFrames.h>
#include <pose_graph_tools_msgs/BowQueries.h>

#include <kimera-vio/backend/VioBackend-definitions.h>
#include <kimera-vio/frontend/StereoVisionImuFrontend-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/mesh/Mesher-definitions.h>

#include "kimera_vio_ros/RosPublishers.h"

namespace VIO {

class RosLoopClosureVisualizer {
 public:
  KIMERA_POINTER_TYPEDEFS(RosLoopClosureVisualizer);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosLoopClosureVisualizer);

 public:
  RosLoopClosureVisualizer();
  ~RosLoopClosureVisualizer() = default;

  void publishLcdOutput(const LcdOutput::ConstPtr& lcd_output);

 private:
  void publishTf(const LcdOutput::ConstPtr& lcd_output);

  void publishOptimizedTrajectory(const LcdOutput::ConstPtr& lcd_output);

  void publishPoseGraph(const LcdOutput::ConstPtr& lcd_output);

  void updateNodesAndEdges(const FrameIDTimestampMap& times,
                           const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values);

  void updateRejectedEdges();

  pose_graph_tools_msgs::PoseGraph getPosegraphMsg();

  // Process bag-of-word vector associated to latest frame
  void processBowQuery();

  // Timer to periodically publish BoW vectors
  void publishTimerCallback(const ros::TimerEvent &event);

  // Service callback to send VLCFrame
  bool VLCServiceCallback(
      pose_graph_tools_msgs::VLCFrameQuery::Request& request,
      pose_graph_tools_msgs::VLCFrameQuery::Response& response);

 private:
  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Unique ID of this robot
  uint16_t robot_id_;

  // ROS publishers
  ros::Publisher trajectory_pub_;
  ros::Publisher posegraph_pub_;
  ros::Publisher odometry_pub_;
  ros::Publisher posegraph_incremental_pub_;
  ros::Publisher bow_query_pub_;
  ros::Publisher vlc_frame_pub_;

  // ROS service
  ros::ServiceServer vlc_frame_server_;

  //! Define tf broadcaster for world to base_link (IMU) and to map (PGO).
  tf::TransformBroadcaster tf_broadcaster_;

  //! Stored pose graph related objects
  std::vector<pose_graph_tools_msgs::PoseGraphEdge> loop_closure_edges_;
  std::vector<pose_graph_tools_msgs::PoseGraphEdge> odometry_edges_;
  std::vector<pose_graph_tools_msgs::PoseGraphEdge> inlier_edges_;
  std::vector<pose_graph_tools_msgs::PoseGraphNode> pose_graph_nodes_;
  std::map<size_t, ros::Time> key_stamped_;

  struct lcd_frame {
    Landmarks keypoints_3d_;
    BearingVectors versors_;
    decltype(LcdOutput::bow_vec_) bow_vec_;
    VIO::OrbDescriptor descriptors_mat_;

    explicit lcd_frame(const LcdOutput& lcd_output)
        : keypoints_3d_(lcd_output.keypoints_3d_),
          versors_(lcd_output.versors_),
          bow_vec_(lcd_output.bow_vec_),
          descriptors_mat_(lcd_output.descriptors_mat_) {}
  };

  std::vector<lcd_frame> frames_;

 private:
  //! Define frame ids for odometry message
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;

  // Number of BoW vectors published in each query msg
  int bow_batch_size_;
  // Include every bow_skip_num_ BoW vectors
  // bow_skip_num=1 means publish all vectors
  int bow_skip_num_;
  // Publish VLC frames
  bool publish_vlc_frames_;

  // BoW queries to different robots
  std::map<uint16_t, pose_graph_tools_msgs::BowQueries> bow_queries_;

  // New VLC frames
  pose_graph_tools_msgs::VLCFrames new_frames_msg_;

  // ROS timer
  ros::Timer publish_timer_;

  // Helper function to convert a VLC frame into a ROS message
  bool getFrameMsg(int pose_id,
                   pose_graph_tools_msgs::VLCFrameMsg& frame_msg) const;
};

}  // namespace VIO
