/**
 * @file   RosDisplay.cpp
 * @brief  Publishes 2D data sent in the display_queue in Kimera. This publishes
 * images at any rate (frame rate, keyframe rate,...).
 * @author Antoni Rosinol
 */
#include "kimera_vio_ros/RosVisualizer.h"

#include <string>

#include <glog/logging.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2/buffer_core.h>

#include <kimera-vio/backend/VioBackend-definitions.h>
#include <kimera-vio/frontend/StereoVisionImuFrontend-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/mesh/Mesh.h>
#include <kimera-vio/mesh/Mesher-definitions.h>
#include <kimera-vio/pipeline/QueueSynchronizer.h>
#include <kimera-vio/visualizer/Visualizer3D.h>

#include "kimera_vio_ros/utils/UtilsRos.h"

namespace VIO {

RosVisualizer::RosVisualizer(const VioParams& vio_params)
    // I'm not sure we use this flag in ROS?
    : Visualizer3D(vio_params.frontend_type_ == FrontendType::kMonoImu
                   ? VisualizationType::kNone
                   : VisualizationType::kMesh2dTo3dSparse),
      nh_(),
      nh_private_("~"),
      image_size_(vio_params.camera_params_.at(0).image_size_),
      image_publishers_(nullptr) {
  //! To publish 2d images
  image_publishers_ = VIO::make_unique<ImagePublishers>(nh_private_);

  // Get ROS params
  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(nh_private_.getParam("world_frame_id", world_frame_id_));
  CHECK(!world_frame_id_.empty());
  CHECK(nh_private_.getParam("map_frame_id", map_frame_id_));
  CHECK(!map_frame_id_.empty());

  // Publishers
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1, true);
  frontend_stats_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("frontend_stats", 1);
  resiliency_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("resiliency", 1);
  imu_bias_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("imu_bias", 1);
  pointcloud_pub_ =
      nh_.advertise<PointCloudXYZRGB>("time_horizon_pointcloud", 1, true);
  mesh_3d_frame_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("mesh", 1, true);
}

VisualizerOutput::UniquePtr RosVisualizer::spinOnce(
    const VisualizerInput& viz_input) {
  publishBackendOutput(viz_input.backend_output_);
  publishFrontendOutput(viz_input.frontend_output_);
  if (viz_input.mesher_output_) publishMesherOutput(viz_input.mesher_output_);
  if (viz_input.lcd_output_)
    lcd_visualizer_.publishLcdOutput(viz_input.lcd_output_);
  // Return empty output, since in ROS, we only publish, not display...
  return VIO::make_unique<VisualizerOutput>();
}

void RosVisualizer::publishBackendOutput(
    const BackendOutput::ConstPtr& output) {
  CHECK(output);
  publishTf(output);
  if (odometry_pub_.getNumSubscribers() > 0) {
    publishState(output);
  }
  if (imu_bias_pub_.getNumSubscribers() > 0) {
    publishImuBias(output);
  }
  if (pointcloud_pub_.getNumSubscribers() > 0) {
    publishTimeHorizonPointCloud(output);
  }
}

void RosVisualizer::publishFrontendOutput(
    const FrontendOutputPacketBase::ConstPtr& output) const {
  CHECK(output);
  if (frontend_stats_pub_.getNumSubscribers() > 0) {
    publishFrontendStats(output);
  }
}

void RosVisualizer::publishMesherOutput(
    const MesherOutput::ConstPtr& output) const {
  CHECK(output);
  if (mesh_3d_frame_pub_.getNumSubscribers() > 0) {
    publishPerFrameMesh3D(output);
  }
}

void RosVisualizer::publishTimeHorizonPointCloud(
    const BackendOutput::ConstPtr& output) const {
  CHECK(output);
  const Timestamp& timestamp = output->timestamp_;
  const PointsWithIdMap& points_with_id = output->landmarks_with_id_map_;
  const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map =
      output->lmk_id_to_lmk_type_map_;

  PointCloudXYZRGB::Ptr msg(new PointCloudXYZRGB);
  msg->header.frame_id = world_frame_id_;
  msg->is_dense = true;
  msg->height = 1;
  msg->width = points_with_id.size();
  msg->points.resize(points_with_id.size());

  bool color_the_cloud = false;
  if (lmk_id_to_lmk_type_map.size() != 0) {
    color_the_cloud = true;
    CHECK_EQ(points_with_id.size(), lmk_id_to_lmk_type_map.size());
  }

  if (points_with_id.size() == 0) {
    // No points to visualize.
    return;
  }

  // Populate cloud structure with 3D points.
  size_t i = 0;
  for (const std::pair<LandmarkId, gtsam::Point3>& id_point : points_with_id) {
    const gtsam::Point3 point_3d = id_point.second;
    msg->points[i].x = static_cast<float>(point_3d.x());
    msg->points[i].y = static_cast<float>(point_3d.y());
    msg->points[i].z = static_cast<float>(point_3d.z());
    if (color_the_cloud) {
      DCHECK(lmk_id_to_lmk_type_map.find(id_point.first) !=
             lmk_id_to_lmk_type_map.end());
      switch (lmk_id_to_lmk_type_map.at(id_point.first)) {
        case LandmarkType::SMART: {
          // point_cloud_color.col(i) = cv::viz::Color::white();
          msg->points[i].r = 0;
          msg->points[i].g = 255;
          msg->points[i].b = 0;
          break;
        }
        case LandmarkType::PROJECTION: {
          // point_cloud_color.col(i) = cv::viz::Color::green();
          msg->points[i].r = 0;
          msg->points[i].g = 0;
          msg->points[i].b = 255;
          break;
        }
        default: {
          // point_cloud_color.col(i) = cv::viz::Color::white();
          msg->points[i].r = 255;
          msg->points[i].g = 0;
          msg->points[i].b = 0;
          break;
        }
      }
    }
    i++;
  }

  ros::Time ros_timestamp;
  ros_timestamp.fromNSec(timestamp);
  pcl_conversions::toPCL(ros_timestamp, msg->header.stamp);
  pointcloud_pub_.publish(msg);
}

void RosVisualizer::publishDebugImage(const Timestamp& timestamp,
                                      const cv::Mat& debug_image) const {
  // CHECK(debug_image.type(), CV_8UC1);
  std_msgs::Header h;
  h.stamp.fromNSec(timestamp);
  h.frame_id = base_link_frame_id_;
  // Copies...
  image_publishers_->publish(
      "mesh_2d", cv_bridge::CvImage(h, "bgr8", debug_image).toImageMsg());
}

void RosVisualizer::publishPerFrameMesh3D(
    const MesherOutput::ConstPtr& output) const {
  CHECK(output);

  const Mesh2D& mesh_2d = output->mesh_2d_;
  const Mesh3D& mesh_3d = output->mesh_3d_;
  size_t number_mesh_2d_polygons = mesh_2d.getNumberOfPolygons();
  size_t mesh_2d_poly_dim = mesh_2d.getMeshPolygonDimension();

  static const size_t cam_width = image_size_.width;
  static const size_t cam_height = image_size_.height;
  DCHECK_GT(cam_width, 0);
  DCHECK_GT(cam_height, 0);

  pcl_msgs::PolygonMesh::Ptr msg(new pcl_msgs::PolygonMesh());
  msg->header.stamp.fromNSec(output->timestamp_);
  msg->header.frame_id = world_frame_id_;

  // Create point cloud to hold vertices.
  pcl::PointCloud<PointNormalUV> cloud;
  cloud.points.reserve(number_mesh_2d_polygons * mesh_2d_poly_dim);
  msg->polygons.reserve(number_mesh_2d_polygons);

  Mesh2D::Polygon polygon;
  for (size_t i = 0; i < number_mesh_2d_polygons; i++) {
    CHECK(mesh_2d.getPolygon(i, &polygon)) << "Could not retrieve 2d polygon.";
    const LandmarkId& lmk0_id = polygon.at(0).getLmkId();
    const LandmarkId& lmk1_id = polygon.at(1).getLmkId();
    const LandmarkId& lmk2_id = polygon.at(2).getLmkId();

    // Returns indices of points in the 3D mesh corresponding to the
    // vertices
    // in the 2D mesh.
    Mesh3D::VertexId p0_id, p1_id, p2_id;
    Mesh3D::VertexType vtx0, vtx1, vtx2;
    if (mesh_3d.getVertex(lmk0_id, &vtx0, &p0_id) &&
        mesh_3d.getVertex(lmk1_id, &vtx1, &p1_id) &&
        mesh_3d.getVertex(lmk2_id, &vtx2, &p2_id)) {
      // Get pixel coordinates of the vertices of the 2D mesh.
      const Vertex2D& px0 = polygon.at(0).getVertexPosition();
      const Vertex2D& px1 = polygon.at(1).getVertexPosition();
      const Vertex2D& px2 = polygon.at(2).getVertexPosition();

      // Get 3D coordinates of the vertices of the 3D mesh.
      const Vertex3D& lmk0_pos = vtx0.getVertexPosition();
      const Vertex3D& lmk1_pos = vtx1.getVertexPosition();
      const Vertex3D& lmk2_pos = vtx2.getVertexPosition();

      // Get normals of the vertices of the 3D mesh.
      const Mesh3D::VertexNormal& normal0 = vtx0.getVertexNormal();
      const Mesh3D::VertexNormal& normal1 = vtx1.getVertexNormal();
      const Mesh3D::VertexNormal& normal2 = vtx2.getVertexNormal();

      // FILL POINTCLOUD
      // clang-format off
      PointNormalUV pn0, pn1, pn2;
      pn0.x = lmk0_pos.x; pn1.x = lmk1_pos.x; pn2.x = lmk2_pos.x;
      pn0.y = lmk0_pos.y; pn1.y = lmk1_pos.y; pn2.y = lmk2_pos.y;
      pn0.z = lmk0_pos.z; pn1.z = lmk1_pos.z; pn2.z = lmk2_pos.z;
      // OpenGL textures range from 0 to 1.
      pn0.u = px0.x / cam_width; pn1.u = px1.x / cam_width; pn2.u = px2.x / cam_width;
      pn0.v = px0.y / cam_height; pn1.v = px1.y / cam_height; pn2.v = px2.y / cam_height;
      pn0.normal_x = normal0.x; pn1.normal_x = normal1.x; pn2.normal_x = normal2.x;
      pn0.normal_y = normal0.y; pn1.normal_y = normal1.y; pn2.normal_y = normal2.y;
      pn0.normal_z = normal0.z; pn1.normal_z = normal1.z; pn2.normal_z = normal2.z;
      // clang-format on

      // TODO(Toni): we are adding repeated vertices!!
      cloud.points.push_back(pn0);
      cloud.points.push_back(pn1);
      cloud.points.push_back(pn2);

      // Store polygon connectivity
      pcl_msgs::Vertices vtx_ii;
      vtx_ii.vertices.resize(3);
      size_t idx = i * mesh_2d_poly_dim;
      // Store connectivity CCW bcs of RVIZ
      vtx_ii.vertices[0] = idx + 2;
      vtx_ii.vertices[1] = idx + 1;
      vtx_ii.vertices[2] = idx;
      msg->polygons.push_back(vtx_ii);
    } else {
      // LOG_EVERY_N(ERROR, 1000) << "Polygon in 2d mesh did not have a
      // corresponding polygon in"
      //                          " 3d mesh!";
    }
  }

  cloud.is_dense = false;
  cloud.width = cloud.points.size();
  cloud.height = 1;
  pcl::toROSMsg(cloud, msg->cloud);

  // NOTE: Header fields need to be filled in after pcl::toROSMsg() call.
  msg->cloud.header = std_msgs::Header();
  msg->cloud.header.stamp = msg->header.stamp;
  msg->cloud.header.frame_id = msg->header.frame_id;

  if (msg->polygons.size() > 0u) {
    mesh_3d_frame_pub_.publish(msg);
  }
}

void RosVisualizer::publishState(const BackendOutput::ConstPtr& output) const {
  CHECK(output);
  // Get latest estimates for odometry.
  const Timestamp& ts = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  const gtsam::Rot3& rotation = pose.rotation();
  const gtsam::Quaternion& quaternion = rotation.toQuaternion();
  const gtsam::Vector3& velocity = output->W_State_Blkf_.velocity_;
  const gtsam::Matrix6& pose_cov =
      gtsam::sub(output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3& vel_cov =
      gtsam::sub(output->state_covariance_lkf_, 6, 9, 6, 9);

  // First publish odometry estimate
  nav_msgs::Odometry odometry_msg;

  // Create header.
  odometry_msg.header.stamp.fromNSec(ts);
  odometry_msg.header.frame_id = world_frame_id_;
  odometry_msg.child_frame_id = base_link_frame_id_;

  // Position
  odometry_msg.pose.pose.position.x = pose.x();
  odometry_msg.pose.pose.position.y = pose.y();
  odometry_msg.pose.pose.position.z = pose.z();

  // Orientation
  odometry_msg.pose.pose.orientation.w = quaternion.w();
  odometry_msg.pose.pose.orientation.x = quaternion.x();
  odometry_msg.pose.pose.orientation.y = quaternion.y();
  odometry_msg.pose.pose.orientation.z = quaternion.z();

  // Remap covariance from GTSAM convention
  // to odometry convention and fill in covariance
  static const std::vector<int> remapping{3, 4, 5, 0, 1, 2};

  // Position covariance first, angular covariance after
  DCHECK_EQ(pose_cov.rows(), remapping.size());
  DCHECK_EQ(pose_cov.rows() * pose_cov.cols(),
            odometry_msg.pose.covariance.size());
  for (int i = 0; i < pose_cov.rows(); i++) {
    for (int j = 0; j < pose_cov.cols(); j++) {
      odometry_msg.pose
          .covariance[remapping[i] * pose_cov.cols() + remapping[j]] =
          pose_cov(i, j);
    }
  }

  // Linear velocities, trivial values for angular
  const gtsam::Matrix3& inversed_rotation = rotation.transpose();
  const Vector3 velocity_body = inversed_rotation * velocity;
  odometry_msg.twist.twist.linear.x = velocity_body(0);
  odometry_msg.twist.twist.linear.y = velocity_body(1);
  odometry_msg.twist.twist.linear.z = velocity_body(2);

  // Velocity covariance: first linear
  // and then angular (trivial values for angular)
  const gtsam::Matrix3 vel_cov_body =
      inversed_rotation.matrix() * vel_cov * rotation.matrix();
  DCHECK_EQ(vel_cov_body.rows(), 3);
  DCHECK_EQ(vel_cov_body.cols(), 3);
  DCHECK_EQ(odometry_msg.twist.covariance.size(), 36);
  for (int i = 0; i < vel_cov_body.rows(); i++) {
    for (int j = 0; j < vel_cov_body.cols(); j++) {
      odometry_msg.twist
          .covariance[i * static_cast<int>(
                              sqrt(odometry_msg.twist.covariance.size())) +
                      j] = vel_cov_body(i, j);
    }
  }
  // Publish message
  odometry_pub_.publish(odometry_msg);
}

void RosVisualizer::publishFrontendStats(
    const FrontendOutputPacketBase::ConstPtr& output) const {
  CHECK(output);

  // Get frontend data for resiliency output
  const DebugTrackerInfo& debug_tracker_info = output->getTrackerInfo();

  // Create message type
  std_msgs::Float64MultiArray frontend_stats_msg;

  // Build Message Layout
  frontend_stats_msg.data.resize(13);
  frontend_stats_msg.data[0] = debug_tracker_info.nrDetectedFeatures_;
  frontend_stats_msg.data[1] = debug_tracker_info.nrTrackerFeatures_;
  frontend_stats_msg.data[2] = debug_tracker_info.nrMonoInliers_;
  frontend_stats_msg.data[3] = debug_tracker_info.nrMonoPutatives_;
  frontend_stats_msg.data[4] = debug_tracker_info.nrStereoInliers_;
  frontend_stats_msg.data[5] = debug_tracker_info.nrStereoPutatives_;
  frontend_stats_msg.data[6] = debug_tracker_info.monoRansacIters_;
  frontend_stats_msg.data[7] = debug_tracker_info.stereoRansacIters_;
  frontend_stats_msg.data[8] = debug_tracker_info.nrValidRKP_;
  frontend_stats_msg.data[9] = debug_tracker_info.nrNoLeftRectRKP_;
  frontend_stats_msg.data[10] = debug_tracker_info.nrNoRightRectRKP_;
  frontend_stats_msg.data[11] = debug_tracker_info.nrNoDepthRKP_;
  frontend_stats_msg.data[12] = debug_tracker_info.nrFailedArunRKP_;
  frontend_stats_msg.layout.dim.resize(1);
  frontend_stats_msg.layout.dim[0].size = frontend_stats_msg.data.size();
  frontend_stats_msg.layout.dim[0].stride = 1;
  frontend_stats_msg.layout.dim[0].label =
      "Frontend: nrDetFeat, nrTrackFeat, nrMoIn, nrMoPu, nrStIn, nrStPu, "
      "moRaIt, stRaIt, nrVaRKP, nrNoLRKP, nrNoRRKP, nrNoDRKP nrFaARKP";

  // Publish Message
  frontend_stats_pub_.publish(frontend_stats_msg);
}

void RosVisualizer::publishResiliency(
    const FrontendOutputPacketBase::ConstPtr& frontend_output,
    const BackendOutput::ConstPtr& backend_output) const {
  CHECK(frontend_output);
  CHECK(backend_output);

  // Get frontend and velocity covariance data for resiliency output
  const DebugTrackerInfo& debug_tracker_info =
      frontend_output->getTrackerInfo();
  const gtsam::Matrix6& pose_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3& vel_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 6, 9, 6, 9);

  // Create message type for quality of KimeraVIO
  std_msgs::Float64MultiArray resiliency_msg;

  // Publishing extra information:
  // cov_v_det and nrStIn should be the most relevant!
  resiliency_msg.layout.dim[0].label =
      "Values: cbrtPDet, cbrtVDet, nrStIn, nrMoIn. "
      "Thresholds : cbrtPDet, cbrtVDet, nrStIn, nrMoIn.";

  CHECK_EQ(pose_cov.size(), 36);
  gtsam::Matrix3 position_cov = gtsam::sub(pose_cov, 3, 6, 3, 6);
  CHECK_EQ(position_cov.size(), 9);

  // Compute eigenvalues and determinant of velocity covariance
  gtsam::Matrix U;
  gtsam::Matrix V;
  gtsam::Vector cov_v_eigv;
  gtsam::svd(vel_cov, U, cov_v_eigv, V);
  CHECK_EQ(cov_v_eigv.size(), 3);

  // Compute eigenvalues and determinant of position covariance
  gtsam::Vector cov_p_eigv;
  gtsam::svd(position_cov, U, cov_p_eigv, V);
  CHECK_EQ(cov_p_eigv.size(), 3);

  // Quality statistics to publish
  resiliency_msg.data.resize(8);
  resiliency_msg.data[0] =
      std::cbrt(cov_p_eigv(0) * cov_p_eigv(1) * cov_p_eigv(2));
  resiliency_msg.data[1] =
      std::cbrt(cov_v_eigv(0) * cov_v_eigv(1) * cov_v_eigv(2));
  resiliency_msg.data[2] = debug_tracker_info.nrStereoInliers_;
  resiliency_msg.data[3] = debug_tracker_info.nrMonoInliers_;

  // Publish thresholds for statistics
  float pos_det_threshold, vel_det_threshold;
  int mono_ransac_theshold, stereo_ransac_threshold;
  CHECK(nh_private_.getParam("velocity_det_threshold", vel_det_threshold));
  CHECK(nh_private_.getParam("position_det_threshold", pos_det_threshold));
  CHECK(
      nh_private_.getParam("stereo_ransac_threshold", stereo_ransac_threshold));
  CHECK(nh_private_.getParam("mono_ransac_threshold", mono_ransac_theshold));
  resiliency_msg.data[4] = pos_det_threshold;
  resiliency_msg.data[5] = vel_det_threshold;
  resiliency_msg.data[6] = stereo_ransac_threshold;
  resiliency_msg.data[7] = mono_ransac_theshold;

  // Build Message Layout
  resiliency_msg.layout.dim.resize(1);
  resiliency_msg.layout.dim[0].size = resiliency_msg.data.size();
  resiliency_msg.layout.dim[0].stride = 1;

  // Publish Message
  resiliency_pub_.publish(resiliency_msg);
}

void RosVisualizer::publishImuBias(
    const BackendOutput::ConstPtr& output) const {
  CHECK(output);

  // Get imu bias to output
  const ImuBias& imu_bias = output->W_State_Blkf_.imu_bias_;
  const Vector3& accel_bias = imu_bias.accelerometer();
  const Vector3& gyro_bias = imu_bias.gyroscope();

  // Create message type
  std_msgs::Float64MultiArray imu_bias_msg;

  // Get Imu Bias to Publish
  imu_bias_msg.data.resize(6);
  imu_bias_msg.data.at(0) = gyro_bias[0];
  imu_bias_msg.data.at(1) = gyro_bias[1];
  imu_bias_msg.data.at(2) = gyro_bias[2];
  imu_bias_msg.data.at(3) = accel_bias[0];
  imu_bias_msg.data.at(4) = accel_bias[1];
  imu_bias_msg.data.at(5) = accel_bias[2];

  // Build Message Layout
  imu_bias_msg.layout.dim.resize(1);
  imu_bias_msg.layout.dim[0].size = imu_bias_msg.data.size();
  imu_bias_msg.layout.dim[0].stride = 1;
  imu_bias_msg.layout.dim[0].label = "Gyro Bias: x,y,z. Accel Bias: x,y,z";

  // Publish Message
  imu_bias_pub_.publish(imu_bias_msg);
}

void RosVisualizer::publishTf(const BackendOutput::ConstPtr& output) {
  CHECK(output);

  const Timestamp& timestamp = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  // const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();
  // Publish base_link TF.
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp.fromNSec(timestamp);
  odom_tf.header.frame_id = world_frame_id_;
  odom_tf.child_frame_id = base_link_frame_id_;

  utils::gtsamPoseToRosTf(pose, &odom_tf.transform);
  tf_broadcaster_.sendTransform(odom_tf);
}

}  // namespace VIO
