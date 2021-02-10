/**
 * @file   RosLoopClosureVisualizer.cpp
 * @brief  Ros interface for the loop closure module
 * @author Yun Chang
 */

#include "kimera_vio_ros/RosLoopClosureVisualizer.h"

#include <string>

#include <glog/logging.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2/buffer_core.h>

#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/pipeline/QueueSynchronizer.h>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosLoopClosureVisualizer::RosLoopClosureVisualizer() : nh_(), nh_private_("~") {
  // Get ROS params
  CHECK(nh_private_.getParam("world_frame_id", world_frame_id_));
  CHECK(!world_frame_id_.empty());
  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(nh_private_.getParam("map_frame_id", map_frame_id_));
  CHECK(!map_frame_id_.empty());

  // Publishers
  trajectory_pub_ = nh_.advertise<nav_msgs::Path>("optimized_trajectory", 1);
  posegraph_pub_ = nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1);
  posegraph_incremental_pub_ =
      nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph_incremental", 1);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("optimized_odometry", 1);
}

void RosLoopClosureVisualizer::publishLcdOutput(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  publishTf(lcd_output);
  if (trajectory_pub_.getNumSubscribers() > 0) {
    publishOptimizedTrajectory(lcd_output);
  }
  if (posegraph_pub_.getNumSubscribers() > 0) {
    publishPoseGraph(lcd_output);
  }
}

void RosLoopClosureVisualizer::publishOptimizedTrajectory(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  // Get pgo-optimized trajectory
  const Timestamp& ts = lcd_output->timestamp_;
  const gtsam::Values& trajectory = lcd_output->states_;
  // Create message type
  nav_msgs::Path path;

  // Fill path poses
  path.poses.reserve(trajectory.size());
  for (size_t i = 0; i < trajectory.size(); i++) {
    gtsam::Pose3 pose = trajectory.at<gtsam::Pose3>(i);
    gtsam::Point3 trans = pose.translation();
    gtsam::Quaternion quat = pose.rotation().toQuaternion();

    geometry_msgs::PoseStamped ps_msg;
    ps_msg.header.frame_id = world_frame_id_;
    ps_msg.pose.position.x = trans.x();
    ps_msg.pose.position.y = trans.y();
    ps_msg.pose.position.z = trans.z();
    ps_msg.pose.orientation.x = quat.x();
    ps_msg.pose.orientation.y = quat.y();
    ps_msg.pose.orientation.z = quat.z();
    ps_msg.pose.orientation.w = quat.w();

    path.poses.push_back(ps_msg);
  }

  // Publish path message
  path.header.stamp.fromNSec(ts);
  path.header.frame_id = world_frame_id_;
  trajectory_pub_.publish(path);

  // publish odometry also
  // Note however this does not update the transform
  // between kimera base_link and world
  // i.e. base_link is still based on unoptimized odometry
  // TODO(Yun) add new frame for optimized base link
  gtsam::Pose3 latest_pose = trajectory.at<gtsam::Pose3>(trajectory.size() - 1);
  gtsam::Point3 trans = latest_pose.translation();
  gtsam::Quaternion quat = latest_pose.rotation().toQuaternion();
  nav_msgs::Odometry odometry_msg;

  // Create header.
  odometry_msg.header.stamp.fromNSec(ts);
  odometry_msg.header.frame_id = world_frame_id_;
  odometry_msg.child_frame_id = base_link_frame_id_;

  // Position
  odometry_msg.pose.pose.position.x = trans.x();
  odometry_msg.pose.pose.position.y = trans.y();
  odometry_msg.pose.pose.position.z = trans.z();

  // Orientation
  odometry_msg.pose.pose.orientation.w = quat.w();
  odometry_msg.pose.pose.orientation.x = quat.x();
  odometry_msg.pose.pose.orientation.y = quat.y();
  odometry_msg.pose.pose.orientation.z = quat.z();

  // Publish message
  odometry_pub_.publish(odometry_msg);
}

// Function used ultimately to visualize loop closure
// And differentiate the inliers and the outliers
void RosLoopClosureVisualizer::updateRejectedEdges() {
  // first update the rejected edges
  for (pose_graph_tools::PoseGraphEdge& loop_closure_edge :
       loop_closure_edges_) {
    bool is_inlier = false;
    for (pose_graph_tools::PoseGraphEdge& inlier_edge : inlier_edges_) {
      if (loop_closure_edge.key_from == inlier_edge.key_from &&
          loop_closure_edge.key_to == inlier_edge.key_to) {
        is_inlier = true;
        continue;
      }
    }
    if (!is_inlier) {
      // set as rejected loop closure
      loop_closure_edge.type =
          pose_graph_tools::PoseGraphEdge::REJECTED_LOOPCLOSE;
    }
  }

  // Then update the loop edges
  for (pose_graph_tools::PoseGraphEdge& inlier_edge : inlier_edges_) {
    bool previously_stored = false;
    for (pose_graph_tools::PoseGraphEdge& loop_closure_edge :
         loop_closure_edges_) {
      if (inlier_edge.key_from == loop_closure_edge.key_from &&
          inlier_edge.key_to == loop_closure_edge.key_to) {
        previously_stored = true;
        continue;
      }
    }
    if (!previously_stored) {
      // add to the vector of all loop clousres
      loop_closure_edges_.push_back(inlier_edge);
    }
  }
}

void RosLoopClosureVisualizer::updateNodesAndEdges(
    const gtsam::NonlinearFactorGraph& nfg,
    const gtsam::Values& values) {
  inlier_edges_.clear();
  odometry_edges_.clear();
  // first store the factors as edges
  for (size_t i = 0; i < nfg.size(); i++) {
    // check if between factor
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
            nfg[i])) {
      // convert to between factor
      const gtsam::BetweenFactor<gtsam::Pose3>& factor =
          *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
              nfg[i]);
      // convert between factor to PoseGraphEdge type
      pose_graph_tools::PoseGraphEdge edge;
      edge.header.frame_id = world_frame_id_;
      edge.key_from = factor.front();
      edge.key_to = factor.back();
      if (edge.key_to == edge.key_from + 1) {  // check if odom
        edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
      } else {
        edge.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
      }
      // transforms - translation
      const gtsam::Point3& translation = factor.measured().translation();
      edge.pose.position.x = translation.x();
      edge.pose.position.y = translation.y();
      edge.pose.position.z = translation.z();
      // transforms - rotation (to quaternion)
      const gtsam::Quaternion& quaternion =
          factor.measured().rotation().toQuaternion();
      edge.pose.orientation.x = quaternion.x();
      edge.pose.orientation.y = quaternion.y();
      edge.pose.orientation.z = quaternion.z();
      edge.pose.orientation.w = quaternion.w();

      // TODO: add covariance
      if (edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
        odometry_edges_.push_back(edge);
      } else {
        inlier_edges_.push_back(edge);
      }
    }
  }

  // update inliers and rejected closures
  updateRejectedEdges();

  pose_graph_nodes_.clear();
  // Then store the values as nodes
  gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    pose_graph_tools::PoseGraphNode node;
    node.key = key_list[i];

    const gtsam::Pose3& value = values.at<gtsam::Pose3>(i);
    const gtsam::Point3& translation = value.translation();
    const gtsam::Quaternion& quaternion = value.rotation().toQuaternion();

    // pose - translation
    node.pose.position.x = translation.x();
    node.pose.position.y = translation.y();
    node.pose.position.z = translation.z();
    // pose - rotation (to quaternion)
    node.pose.orientation.x = quaternion.x();
    node.pose.orientation.y = quaternion.y();
    node.pose.orientation.z = quaternion.z();
    node.pose.orientation.w = quaternion.w();

    pose_graph_nodes_.push_back(node);
  }

  return;
}

pose_graph_tools::PoseGraph RosLoopClosureVisualizer::getPosegraphMsg() {
  // pose graph getter
  pose_graph_tools::PoseGraph pose_graph;
  pose_graph.edges = odometry_edges_;  // add odometry edges to pg
  // then add loop closure edges to pg
  pose_graph.edges.insert(pose_graph.edges.end(),
                          loop_closure_edges_.begin(),
                          loop_closure_edges_.end());
  // then add the nodes
  pose_graph.nodes = pose_graph_nodes_;

  return pose_graph;
}

void RosLoopClosureVisualizer::publishPoseGraph(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  // Get the factor graph
  const Timestamp& ts = lcd_output->timestamp_;
  const gtsam::NonlinearFactorGraph& nfg = lcd_output->nfg_;
  const gtsam::Values& values = lcd_output->states_;
  updateNodesAndEdges(nfg, values);
  pose_graph_tools::PoseGraph graph = getPosegraphMsg();
  graph.header.stamp.fromNSec(ts);
  graph.header.frame_id = world_frame_id_;
  posegraph_pub_.publish(graph);

  // Construct and publish incremental pose graph
  // with the newest odometry and loop closure edges
  // Note that this means we require the lcd module to
  // publish every frame (not just when lc found)
  // TODO(Yun) publish keyed-odometry instead in RosVisualizer
  // (Or create key in receiver - but need to make sure no msg drop)
  if (odometry_edges_.size() > 0) {
    pose_graph_tools::PoseGraph incremental_graph;
    pose_graph_tools::PoseGraphEdge last_odom_edge =
        odometry_edges_.at(odometry_edges_.size() - 1);
    last_odom_edge.header.stamp.fromNSec(ts);
    incremental_graph.edges.push_back(last_odom_edge);
    incremental_graph.nodes.push_back(
        pose_graph_nodes_.at(pose_graph_nodes_.size() - 2));
    incremental_graph.nodes.push_back(
        pose_graph_nodes_.at(pose_graph_nodes_.size() - 1));
    if (lcd_output->is_loop_closure_) {
      // Not directly taking the last lc_edge to bypass kimera-rpgo
      gtsam::Pose3 lc_transform = lcd_output->relative_pose_;
      pose_graph_tools::PoseGraphEdge last_lc_edge;

      const gtsam::Point3& translation = lc_transform.translation();
      const gtsam::Quaternion& quaternion =
          lc_transform.rotation().toQuaternion();
      last_lc_edge.key_from = lcd_output->id_match_,
      last_lc_edge.key_to = lcd_output->id_recent_;
      last_lc_edge.pose.position.x = translation.x();
      last_lc_edge.pose.position.y = translation.y();
      last_lc_edge.pose.position.z = translation.z();
      last_lc_edge.pose.orientation.x = quaternion.x();
      last_lc_edge.pose.orientation.y = quaternion.y();
      last_lc_edge.pose.orientation.z = quaternion.z();
      last_lc_edge.pose.orientation.w = quaternion.w();
      last_lc_edge.header.stamp.fromNSec(ts);
      incremental_graph.edges.push_back(last_lc_edge);
    }
    incremental_graph.header.stamp.fromNSec(ts);
    incremental_graph.header.frame_id = world_frame_id_;
    posegraph_incremental_pub_.publish(incremental_graph);
  }
}

void RosLoopClosureVisualizer::publishTf(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  const Timestamp& ts = lcd_output->timestamp_;
  const gtsam::Pose3& w_Pose_map = lcd_output->W_Pose_Map_;
  const gtsam::Quaternion& w_Quat_map = w_Pose_map.rotation().toQuaternion();
  // Publish map TF.
  geometry_msgs::TransformStamped map_tf;
  map_tf.header.stamp.fromNSec(ts);
  map_tf.header.frame_id = world_frame_id_;
  map_tf.child_frame_id = map_frame_id_;
  utils::gtsamPoseToRosTf(w_Pose_map, &map_tf.transform);
  tf_broadcaster_.sendTransform(map_tf);
}

}  // namespace VIO
