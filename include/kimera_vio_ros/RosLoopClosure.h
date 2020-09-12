/**
 * @file   RosLoopClosure.h
 * @brief  Equivalent Kimera Loop Closure Detector but in ROS. Publishes Loop
 * closure and pose graph data to ROS.
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

#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>

#include <kimera-vio/backend/VioBackEnd-definitions.h>
#include <kimera-vio/frontend/StereoVisionFrontEnd-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/mesh/Mesher-definitions.h>

#include "kimera_vio_ros/RosPublishers.h"

namespace VIO {

class RosLoopClosure : public LoopClosureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(RosLoopClosure);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosLoopClosure);

 public:
  RosLoopClosure(const LoopClosureDetectorParams& lcd_params, bool log_output);
  virtual ~RosLoopClosure() = default;

 public:
  /**
   * @brief spinOnce
   * Spins the display once to render the visualizer output.
   * @param viz_input
   */
  LcdOutput::UniquePtr spinOnce(const LcdInput& lcd_input) override;

 protected:
  // Publish all outputs for LCD
  virtual void publishLcdOutput(const LcdOutput::ConstPtr& lcd_output);

 private:
  void publishTf(const LcdOutput::ConstPtr& lcd_output);

  void publishOptimizedTrajectory(const LcdOutput::ConstPtr& lcd_output);

  void publishPoseGraph(const LcdOutput::ConstPtr& lcd_output);

  void updateNodesAndEdges(const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values);

  void updateRejectedEdges();

  void publishNewNodesAndEdges(const LcdOutput::ConstPtr& lcd_output);

  pose_graph_tools::PoseGraph getPosegraphMsg();

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

  //! Define tf broadcaster for world to base_link (IMU) and to map (PGO).
  tf::TransformBroadcaster tf_broadcaster_;

  //! Stored pose graph related objects
  std::vector<pose_graph_tools::PoseGraphEdge> loop_closure_edges_;
  std::vector<pose_graph_tools::PoseGraphEdge> odometry_edges_;
  std::vector<pose_graph_tools::PoseGraphEdge> inlier_edges_;
  std::vector<pose_graph_tools::PoseGraphNode> pose_graph_nodes_;

 private:
  //! Define frame ids for odometry message
  std::string world_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;
};

}  // namespace VIO