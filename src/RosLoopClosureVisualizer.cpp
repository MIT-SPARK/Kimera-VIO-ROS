/**
 * @file   RosLoopClosureVisualizer.cpp
 * @brief  Ros interface for the loop closure module
 * @author Yun Chang
 */

#include "kimera_vio_ros/RosLoopClosureVisualizer.h"

#include <string>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2/buffer_core.h>

#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/pipeline/QueueSynchronizer.h>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosLoopClosureVisualizer::RosLoopClosureVisualizer() : 
  nh_(), 
  nh_private_("~"),
  bow_batch_size_(5),
  bow_skip_num_(1),
  publish_vlc_frames_(true) {
  // Get ROS params
  CHECK(nh_private_.getParam("odom_frame_id", odom_frame_id_));
  CHECK(!odom_frame_id_.empty());
  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(nh_private_.getParam("map_frame_id", map_frame_id_));
  CHECK(!map_frame_id_.empty());
  int robot_id_in_;
  CHECK(nh_private_.getParam("robot_id", robot_id_in_));
  CHECK(robot_id_in_ >= 0);
  robot_id_ = robot_id_in_;
  nh_private_.param("bow_batch_size", bow_batch_size_, 5);
  nh_private_.param("bow_skip_num", bow_skip_num_, 1);
  nh_private_.param("publish_vlc_frames", publish_vlc_frames_, true);
  ROS_INFO("BoW vector batch size: %d", bow_batch_size_);
  ROS_INFO("BoW vector skip num: %d", bow_skip_num_);
  ROS_INFO("Publish VLC frames: %d", publish_vlc_frames_);

  // Publishers
  trajectory_pub_ = nh_.advertise<nav_msgs::Path>("optimized_trajectory", 1);
  posegraph_pub_ =
      nh_.advertise<pose_graph_tools_msgs::PoseGraph>("pose_graph", 1);
  posegraph_incremental_pub_ = nh_.advertise<pose_graph_tools_msgs::PoseGraph>(
      "pose_graph_incremental", 1000);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("optimized_odometry", 1);
  bow_query_pub_ =
      nh_.advertise<pose_graph_tools_msgs::BowQueries>("bow_query", 1000);
  vlc_frame_pub_ =
      nh_.advertise<pose_graph_tools_msgs::VLCFrames>("vlc_frames", 100);
  // Service
  vlc_frame_server_ = nh_.advertiseService(
      "vlc_frame_query", &RosLoopClosureVisualizer::VLCServiceCallback, this);

  // Initialize BoW queries
  for (uint16_t robot_id = 0; robot_id <= robot_id_; ++robot_id) {
    pose_graph_tools_msgs::BowQueries msg;
    msg.destination_robot_id = robot_id;
    bow_queries_[robot_id] = msg;
  }

  publish_timer_ = nh_.createTimer(ros::Duration(1.0),
                                   &RosLoopClosureVisualizer::publishTimerCallback,
                                   this);

  new_frames_msg_.destination_robot_id = robot_id_;
}

void RosLoopClosureVisualizer::publishLcdOutput(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);
  frames_.push_back(lcd_frame(*lcd_output));

  processBowQuery();
  if (publish_vlc_frames_) {
    size_t pose_id = frames_.size() - 1;
    pose_graph_tools_msgs::VLCFrameMsg frame_msg;
    if (getFrameMsg(pose_id, frame_msg)) {
      new_frames_msg_.frames.push_back(frame_msg);
    }
  }

  publishTf(lcd_output);
  if (trajectory_pub_.getNumSubscribers() > 0) {
    publishOptimizedTrajectory(lcd_output);
  }
  if (posegraph_pub_.getNumSubscribers() > 0 ||
      posegraph_incremental_pub_.getNumSubscribers() > 0) {
    publishPoseGraph(lcd_output);
  }
}

void RosLoopClosureVisualizer::publishOptimizedTrajectory(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  // Get pgo-optimized trajectory
  const Timestamp& ts = lcd_output->timestamp_;
  const FrameIDTimestampMap& times = lcd_output->timestamp_map_;
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
    CHECK(times.count(i));
    ps_msg.header.stamp.fromNSec(times.at(i));
    ps_msg.header.frame_id = map_frame_id_;
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
  path.header.frame_id = map_frame_id_;
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
  odometry_msg.header.frame_id = map_frame_id_;
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
  for (pose_graph_tools_msgs::PoseGraphEdge& loop_closure_edge :
       loop_closure_edges_) {
    bool is_inlier = false;
    for (pose_graph_tools_msgs::PoseGraphEdge& inlier_edge : inlier_edges_) {
      if (loop_closure_edge.key_from == inlier_edge.key_from &&
          loop_closure_edge.key_to == inlier_edge.key_to) {
        is_inlier = true;
        continue;
      }
    }
    if (!is_inlier) {
      // set as rejected loop closure
      loop_closure_edge.type =
          pose_graph_tools_msgs::PoseGraphEdge::REJECTED_LOOPCLOSE;
    }
  }

  // Then update the loop edges
  for (pose_graph_tools_msgs::PoseGraphEdge& inlier_edge : inlier_edges_) {
    bool previously_stored = false;
    for (pose_graph_tools_msgs::PoseGraphEdge& loop_closure_edge :
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

using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;

void RosLoopClosureVisualizer::updateNodesAndEdges(
    const FrameIDTimestampMap& times,
    const gtsam::NonlinearFactorGraph& nfg,
    const gtsam::Values& values) {
  inlier_edges_.clear();
  odometry_edges_.clear();
  // first store the factors as edges
  for (size_t i = 0; i < nfg.size(); i++) {
    const auto factor = dynamic_cast<const PoseBetween*>(nfg[i].get());
    // check if between factor
    if (factor) {
      // convert between factor to PoseGraphEdge type
      pose_graph_tools_msgs::PoseGraphEdge edge;
      edge.header.frame_id = map_frame_id_;
      edge.key_from = factor->front();
      edge.key_to = factor->back();
      edge.robot_from = robot_id_;
      edge.robot_to = robot_id_;
      if (edge.key_to == edge.key_from + 1) {  // check if odom
        edge.type = pose_graph_tools_msgs::PoseGraphEdge::ODOM;
      } else {
        edge.type = pose_graph_tools_msgs::PoseGraphEdge::LOOPCLOSE;
      }
      // transforms - translation
      const gtsam::Point3& translation = factor->measured().translation();
      edge.pose.position.x = translation.x();
      edge.pose.position.y = translation.y();
      edge.pose.position.z = translation.z();
      // transforms - rotation (to quaternion)
      const gtsam::Quaternion& quaternion =
          factor->measured().rotation().toQuaternion();
      edge.pose.orientation.x = quaternion.x();
      edge.pose.orientation.y = quaternion.y();
      edge.pose.orientation.z = quaternion.z();
      edge.pose.orientation.w = quaternion.w();

      // TODO: add covariance
      if (edge.type == pose_graph_tools_msgs::PoseGraphEdge::ODOM) {
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
    pose_graph_tools_msgs::PoseGraphNode node;
    node.key = key_list[i];
    node.robot_id = robot_id_;

    const uint64_t frame_id = gtsam::Symbol(node.key).index();
    CHECK(times.count(frame_id));
    node.header.stamp.fromNSec(times.at(frame_id));

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

pose_graph_tools_msgs::PoseGraph RosLoopClosureVisualizer::getPosegraphMsg() {
  // pose graph getter
  pose_graph_tools_msgs::PoseGraph pose_graph;
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
  updateNodesAndEdges(lcd_output->timestamp_map_, nfg, values);
  pose_graph_tools_msgs::PoseGraph graph = getPosegraphMsg();
  graph.header.stamp.fromNSec(ts);
  graph.header.frame_id = map_frame_id_;
  posegraph_pub_.publish(graph);

  // Construct and publish incremental pose graph
  // with the newest odometry and loop closure edges
  // Note that this means we require the lcd module to
  // publish every frame (not just when lc found)
  // TODO(Yun) publish keyed-odometry instead in RosVisualizer
  // (Or create key in receiver - but need to make sure no msg drop)
  if (odometry_edges_.size() > 0) {
    pose_graph_tools_msgs::PoseGraph incremental_graph;
    pose_graph_tools_msgs::PoseGraphEdge last_odom_edge =
        odometry_edges_.at(odometry_edges_.size() - 1);
    last_odom_edge.header.stamp.fromNSec(ts);
    last_odom_edge.type = pose_graph_tools_msgs::PoseGraphEdge::ODOM;
    last_odom_edge.robot_from = robot_id_;
    last_odom_edge.robot_to = robot_id_;
    incremental_graph.edges.push_back(last_odom_edge);
    incremental_graph.nodes.push_back(
        pose_graph_nodes_.at(pose_graph_nodes_.size() - 2));
    incremental_graph.nodes.push_back(
        pose_graph_nodes_.at(pose_graph_nodes_.size() - 1));
    if (lcd_output->is_loop_closure_) {
      // Not directly taking the last lc_edge to bypass kimera-rpgo
      gtsam::Pose3 lc_transform = lcd_output->relative_pose_;
      pose_graph_tools_msgs::PoseGraphEdge last_lc_edge;

      const gtsam::Point3& translation = lc_transform.translation();
      const gtsam::Quaternion& quaternion =
          lc_transform.rotation().toQuaternion();
      last_lc_edge.key_from = lcd_output->id_match_,
      last_lc_edge.key_to = lcd_output->id_recent_;
      last_lc_edge.robot_from = robot_id_;
      last_lc_edge.robot_to = robot_id_;
      last_lc_edge.pose.position.x = translation.x();
      last_lc_edge.pose.position.y = translation.y();
      last_lc_edge.pose.position.z = translation.z();
      last_lc_edge.pose.orientation.x = quaternion.x();
      last_lc_edge.pose.orientation.y = quaternion.y();
      last_lc_edge.pose.orientation.z = quaternion.z();
      last_lc_edge.pose.orientation.w = quaternion.w();
      last_lc_edge.header.stamp.fromNSec(ts);
      last_lc_edge.type = pose_graph_tools_msgs::PoseGraphEdge::LOOPCLOSE;
      incremental_graph.edges.push_back(last_lc_edge);
      loop_closure_edges_.push_back(last_lc_edge);
      incremental_graph.edges.push_back(last_lc_edge);
    }
    incremental_graph.header.stamp.fromNSec(ts);
    incremental_graph.header.frame_id = map_frame_id_;
    posegraph_incremental_pub_.publish(incremental_graph);
  }
}

void RosLoopClosureVisualizer::publishTf(
    const LcdOutput::ConstPtr& lcd_output) {
  CHECK(lcd_output);

  const Timestamp& ts = lcd_output->timestamp_;
  const gtsam::Pose3& map_Pose_odom = lcd_output->Map_Pose_Odom_;
  // Publish map TF.
  geometry_msgs::TransformStamped map_tf;
  map_tf.header.stamp.fromNSec(ts);
  map_tf.header.frame_id = map_frame_id_;
  map_tf.child_frame_id = odom_frame_id_;
  utils::gtsamPoseToRosTf(map_Pose_odom, &map_tf.transform);
  tf_broadcaster_.sendTransform(map_tf);
}

void RosLoopClosureVisualizer::processBowQuery() {
  if (frames_.size() == 0) 
    return;
  size_t pose_id = frames_.size() - 1;
  if (pose_id % bow_skip_num_ != 0)
    return;

  pose_graph_tools_msgs::BowVector bow_vec_msg;
  for (auto it = frames_.back().bow_vec_.begin();
       it != frames_.back().bow_vec_.end();
       ++it) {
    bow_vec_msg.word_ids.push_back(it->first);
    bow_vec_msg.word_values.push_back(it->second);
  }
  pose_graph_tools_msgs::BowQuery bow_msg;
  bow_msg.robot_id = robot_id_;
  bow_msg.pose_id = pose_id;
  bow_msg.bow_vector = bow_vec_msg;
  for (uint16_t robot_id = 0; robot_id <= robot_id_; ++robot_id) {
    bow_queries_[robot_id].queries.push_back(bow_msg);
  }
}

void RosLoopClosureVisualizer::publishTimerCallback(const ros::TimerEvent& event) {
  // Publish new BoW vectors to myself
  // This won't incur any real communication
  if (bow_queries_[robot_id_].queries.size() >= bow_batch_size_) {
    bow_query_pub_.publish(bow_queries_[robot_id_]);
    bow_queries_[robot_id_].queries.clear();
  }

  // Select a peer robot to publish BoW
  uint16_t selected_robot_id = 0;
  size_t selected_batch_size = 0;
  for (uint16_t robot_id = 0; robot_id < robot_id_; ++robot_id) {
    if (bow_queries_[robot_id].queries.size() >= selected_batch_size) {
      selected_robot_id = robot_id;
      selected_batch_size = bow_queries_[robot_id].queries.size();
    }
  }

  if (selected_batch_size >= bow_batch_size_) {
    ROS_INFO("Published %zu BoW vectors to robot %hu.", selected_batch_size, selected_robot_id);
    bow_query_pub_.publish(bow_queries_[selected_robot_id]);
    bow_queries_[selected_robot_id].queries.clear();
  }

  if (new_frames_msg_.frames.size() >= 50) {
    vlc_frame_pub_.publish(new_frames_msg_);
    new_frames_msg_.frames.clear();
  }
}

bool RosLoopClosureVisualizer::VLCServiceCallback(
    pose_graph_tools_msgs::VLCFrameQuery::Request& request,
    pose_graph_tools_msgs::VLCFrameQuery::Response& response) {
  // Check requested frames belong to this robot
  CHECK(request.robot_id == robot_id_);
  response.frames.clear();

  // Loop through requested pose ids
  for (const auto& pose_id : request.pose_ids) {
    pose_graph_tools_msgs::VLCFrameMsg frame_msg;
    if (!getFrameMsg(pose_id, frame_msg)) {
      ROS_ERROR_STREAM("Requested frame " << pose_id << " does not exist!");
      continue;
    }
    response.frames.push_back(frame_msg);
  }

  return true;
}

bool RosLoopClosureVisualizer::getFrameMsg(
    int pose_id,
    pose_graph_tools_msgs::VLCFrameMsg& frame_msg) const {
  if (pose_id >= frames_.size()) {
    return false;
  }
  const auto& frame = frames_[pose_id];

  frame_msg.robot_id = robot_id_;
  frame_msg.pose_id = pose_id;

  // Convert keypoints
  pcl::PointCloud<pcl::PointXYZ> versors;
  for (size_t i = 0; i < frame.keypoints_3d_.size(); ++i) {
    // Push bearing vector
    gtsam::Vector3 v_ = frame.versors_[i];
    pcl::PointXYZ v(v_(0), v_(1), v_(2));
    versors.push_back(v);
    // Push keypoint depth
    gtsam::Vector3 p_ = frame.keypoints_3d_[i];
    if (p_.norm() < 1e-3) {
      // This 3D keypoint is not valid
      frame_msg.depths.push_back(0);
    } else {
      // We have valid 3D keypoint and the depth is given by the z component
      // See sparseStereoReconstruction function in Stereo Matcher in
      // Kimera-VIO.
      frame_msg.depths.push_back(p_[2]);
    }
  }
  pcl::toROSMsg(versors, frame_msg.versors);

  // Convert descriptors
  cv_bridge::CvImage cv_img;
  // cv_img.header   = in_msg->header; // Yulun: need to set header
  // explicitly?
  cv_img.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  cv_img.image = frame.descriptors_mat_;
  cv_img.toImageMsg(frame_msg.descriptors_mat);

  return true;
}

}  // namespace VIO
