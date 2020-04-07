/**
 * @file   RosDataProviderInterface.cpp
 * @brief  Base class for ROS wrappers for Kimera-VIO.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/RosDataProviderInterface.h"

#include <string>
#include <vector>

#include <glog/logging.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>

#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/pipeline/PipelineModule.h>
#include <kimera-vio/pipeline/QueueSynchronizer.h>
#include <kimera-vio/visualizer/Visualizer3D.h>

namespace VIO {

RosDataProviderInterface::RosDataProviderInterface(const VioParams& vio_params)
    : DataProviderInterface(),
      it_(nullptr),
      nh_(),
      nh_private_("~"),
      backend_output_queue_("Backend output ROS"),
      frame_rate_frontend_output_queue_("Frame Rate Frontend output ROS"),
      keyframe_rate_frontend_output_queue_("Keyframe Rate Frontend output ROS"),
      mesher_output_queue_("Mesher output ROS"),
      lcd_output_queue_("LCD output ROS"),
      vio_params_(vio_params) {
  VLOG(1) << "Initializing RosDataProviderInterface.";

  // Print parameters to check.
  if (VLOG_IS_ON(1)) printParsedParams();

  it_ = VIO::make_unique<image_transport::ImageTransport>(nh_);

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
  resiliency_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("resiliency", 1);
  imu_bias_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("imu_bias", 1);
  trajectory_pub_ = nh_.advertise<nav_msgs::Path>("optimized_trajectory", 1);
  posegraph_pub_ = nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1);
  pointcloud_pub_ =
      nh_.advertise<PointCloudXYZRGB>("time_horizon_pointcloud", 1, true);
  mesh_3d_frame_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("mesh", 1, true);
  debug_img_pub_ = it_->advertise("debug_mesh_img/image_raw", 1, true);
  feature_tracks_pub_ = it_->advertise("feature_tracks/image_raw", 1, true);

  publishStaticTf(vio_params_.camera_params_.at(0).body_Pose_cam_,
                  base_link_frame_id_,
                  left_cam_frame_id_);
  publishStaticTf(vio_params_.camera_params_.at(1).body_Pose_cam_,
                  base_link_frame_id_,
                  right_cam_frame_id_);
}

RosDataProviderInterface::~RosDataProviderInterface() {
  VLOG(1) << "RosBaseDataProvider destructor called.";
}

// TODO(marcus): From this documentation
//  (http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
//  we should be using toCvShare to get a CvImage pointer. However, this is a
//  shared pointer and the ROS message data isn't freed. This means anyone else
//  can modify this from another cb and our version will change too. Even the
//  const isn't enough because people can just copy that and still have access
//  to underlying data. Neet a smarter way to move these around yet make this
//  faster.
const cv::Mat RosDataProviderInterface::readRosImage(
    const sensor_msgs::ImageConstPtr& img_msg) const {
  CHECK(img_msg);
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  CHECK(cv_ptr);
  const cv::Mat img_const = cv_ptr->image;  // Don't modify shared image in ROS.
  cv::Mat converted_img(img_const.size(), CV_8U);
  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
    return converted_img;
  } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    LOG_EVERY_N(WARNING, 10) << "Converting image...";
    cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
    return converted_img;
  } else {
    CHECK_EQ(cv_ptr->encoding, sensor_msgs::image_encodings::MONO8)
        << "Expected image with MONO8, BGR8, or RGB8 encoding."
           "Add in here more conversions if you wish.";
    return img_const;
  }
}

const cv::Mat RosDataProviderInterface::readRosDepthImage(
    const sensor_msgs::ImageConstPtr& img_msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // TODO(Toni): here we should consider using toCvShare...
    cv_ptr =
        cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  cv::Mat img_depth = cv_ptr->image;
  if (img_depth.type() != CV_16UC1) {
    LOG_EVERY_N(WARNING, 10) << "Converting depth image...";
    img_depth.convertTo(img_depth, CV_16UC1);
  }
  return img_depth;
}

void RosDataProviderInterface::publishBackendOutput(
    const BackendOutput::Ptr& output) {
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

void RosDataProviderInterface::publishFrontendOutput(
    const FrontendOutput::Ptr& output) const {
  CHECK(output);
  if (frontend_stats_pub_.getNumSubscribers() > 0) {
    publishFrontendStats(output);
  }
  if (feature_tracks_pub_.getNumSubscribers() > 0) {
    if (!output->feature_tracks_.empty()) {
      std_msgs::Header h;
      h.stamp.fromNSec(output->timestamp_);
      h.frame_id = base_link_frame_id_;
      // Copies...
      feature_tracks_pub_.publish(
          cv_bridge::CvImage(h, "bgr8", output->feature_tracks_).toImageMsg());
    } else {
      LOG(ERROR) << feature_tracks_pub_.getNumSubscribers()
                 << " nodes subscribed to the feature tracks topic, but the "
                    "feature tracks image is empty. Did you set the gflag "
                    "visualize_feature_tracks in Kimera-VIO to true?";
    }
  }
}

void RosDataProviderInterface::publishMesherOutput(
    const MesherOutput::Ptr& output) const {
  CHECK(output);
  if (mesh_3d_frame_pub_.getNumSubscribers() > 0) {
    publishPerFrameMesh3D(output);
  }
}

bool RosDataProviderInterface::publishSyncedOutputs() {
  // First acquire a backend output packet, as it is slowest.
  BackendOutput::Ptr backend_output = nullptr;
  if (backend_output_queue_.pop(backend_output)) {
    CHECK(backend_output);
    publishBackendOutput(backend_output);
    const Timestamp& ts = backend_output->timestamp_;

    FrontendOutput::Ptr frontend_output = nullptr;
    bool get_frontend =
        SimpleQueueSynchronizer<FrontendOutput::Ptr>::getInstance().syncQueue(
            ts,
            &keyframe_rate_frontend_output_queue_,
            &frontend_output,
            "RosDataProvider");
    CHECK(frontend_output);

    MesherOutput::Ptr mesher_output = nullptr;
    bool get_mesher =
        SimpleQueueSynchronizer<MesherOutput::Ptr>::getInstance().syncQueue(
            ts, &mesher_output_queue_, &mesher_output, "RosDataProvider");
    if (mesher_output) {
      publishMesherOutput(mesher_output);
    }

    if (frontend_output && mesher_output) {
      // Publish 2d mesh debug image
      if (debug_img_pub_.getNumSubscribers() > 0) {
        cv::Mat mesh_2d_img = Visualizer3D::visualizeMesh2D(
            mesher_output->mesh_2d_for_viz_,
            frontend_output->stereo_frame_lkf_.getLeftFrame().img_);
        publishDebugImage(backend_output->timestamp_, mesh_2d_img);
      }
    }

    if (frontend_output && backend_output) {
      // Publish Resiliency
      if (resiliency_pub_.getNumSubscribers() > 0) {
        publishResiliency(frontend_output, backend_output);
      }
    }

    return get_frontend && get_mesher;
  }

  return false;
}

void RosDataProviderInterface::publishLcdOutput(
    const LcdOutput::Ptr& lcd_output) {
  CHECK(lcd_output);

  publishTf(lcd_output);
  if (trajectory_pub_.getNumSubscribers() > 0) {
    publishOptimizedTrajectory(lcd_output);
  }
  if (posegraph_pub_.getNumSubscribers() > 0) {
    publishPoseGraph(lcd_output);
  }
}

void RosDataProviderInterface::publishTimeHorizonPointCloud(
    const BackendOutput::Ptr& output) const {
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

void RosDataProviderInterface::publishDebugImage(
    const Timestamp& timestamp,
    const cv::Mat& debug_image) const {
  // CHECK(debug_image.type(), CV_8UC1);
  std_msgs::Header h;
  h.stamp.fromNSec(timestamp);
  h.frame_id = base_link_frame_id_;
  // Copies...
  debug_img_pub_.publish(
      cv_bridge::CvImage(h, "bgr8", debug_image).toImageMsg());
}

// void RosBaseDataProvider::publishTimeHorizonMesh3D(
//    const MesherOutput::Ptr& output) {
//  const Mesh3D& mesh_3d = output->mesh_3d_;
//  size_t number_mesh_3d_polygons = mesh_3d.getNumberOfPolygons();
//}

void RosDataProviderInterface::publishPerFrameMesh3D(
    const MesherOutput::Ptr& output) const {
  CHECK(output);

  const Mesh2D& mesh_2d = output->mesh_2d_;
  const Mesh3D& mesh_3d = output->mesh_3d_;
  size_t number_mesh_2d_polygons = mesh_2d.getNumberOfPolygons();
  size_t mesh_2d_poly_dim = mesh_2d.getMeshPolygonDimension();

  static const size_t cam_width =
      vio_params_.camera_params_.at(0).image_size_.width;
  static const size_t cam_height =
      vio_params_.camera_params_.at(0).image_size_.height;
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
    int p0_id, p1_id, p2_id;
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

  if (msg->polygons.size() > 0) {
    mesh_3d_frame_pub_.publish(msg);
  }

  return;
}  // namespace VIO

void RosDataProviderInterface::publishState(
    const BackendOutput::Ptr& output) const {
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

void RosDataProviderInterface::publishTf(const BackendOutput::Ptr& output) {
  CHECK(output);

  const Timestamp& timestamp = output->timestamp_;
  const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
  const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();
  // Publish base_link TF.
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp.fromNSec(timestamp);
  odom_tf.header.frame_id = world_frame_id_;
  odom_tf.child_frame_id = base_link_frame_id_;

  poseToMsgTF(pose, &odom_tf.transform);
  tf_broadcaster_.sendTransform(odom_tf);
}

void RosDataProviderInterface::publishFrontendStats(
    const FrontendOutput::Ptr& output) const {
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
      "FrontEnd: nrDetFeat, nrTrackFeat, nrMoIn, nrMoPu, nrStIn, nrStPu, "
      "moRaIt, stRaIt, nrVaRKP, nrNoLRKP, nrNoRRKP, nrNoDRKP nrFaARKP";

  // Publish Message
  frontend_stats_pub_.publish(frontend_stats_msg);
}

void RosDataProviderInterface::publishResiliency(
    const FrontendOutput::Ptr& frontend_output,
    const BackendOutput::Ptr& backend_output) const {
  CHECK(frontend_output);
  CHECK(backend_output);

  // Get frontend and velocity covariance data for resiliency output
  const DebugTrackerInfo& debug_tracker_info =
      frontend_output->getTrackerInfo();
  const gtsam::Matrix6& pose_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 0, 6, 0, 6);
  const gtsam::Matrix3& vel_cov =
      gtsam::sub(backend_output->state_covariance_lkf_, 6, 9, 6, 9);

  // Create message type for quality of SparkVIO
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

void RosDataProviderInterface::publishImuBias(
    const BackendOutput::Ptr& output) const {
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

void RosDataProviderInterface::publishOptimizedTrajectory(
    const LcdOutput::Ptr& lcd_output) const {
  CHECK(lcd_output);

  // Get pgo-optimized trajectory
  const Timestamp& ts = lcd_output->timestamp_kf_;
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
}

void RosDataProviderInterface::updateRejectedEdges() {
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

void RosDataProviderInterface::updateNodesAndEdges(
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

pose_graph_tools::PoseGraph RosDataProviderInterface::getPosegraphMsg() {
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

void RosDataProviderInterface::publishPoseGraph(
    const LcdOutput::Ptr& lcd_output) {
  CHECK(lcd_output);

  // Get the factor graph
  const Timestamp& ts = lcd_output->timestamp_kf_;
  const gtsam::NonlinearFactorGraph& nfg = lcd_output->nfg_;
  const gtsam::Values& values = lcd_output->states_;
  updateNodesAndEdges(nfg, values);
  pose_graph_tools::PoseGraph graph = getPosegraphMsg();
  graph.header.stamp.fromNSec(ts);
  graph.header.frame_id = world_frame_id_;
  posegraph_pub_.publish(graph);
}

void RosDataProviderInterface::publishTf(const LcdOutput::Ptr& lcd_output) {
  CHECK(lcd_output);

  const Timestamp& ts = lcd_output->timestamp_kf_;
  const gtsam::Pose3& w_Pose_map = lcd_output->W_Pose_Map_;
  const gtsam::Quaternion& w_Quat_map = w_Pose_map.rotation().toQuaternion();
  // Publish map TF.
  geometry_msgs::TransformStamped map_tf;
  map_tf.header.stamp.fromNSec(ts);
  map_tf.header.frame_id = world_frame_id_;
  map_tf.child_frame_id = map_frame_id_;
  poseToMsgTF(w_Pose_map, &map_tf.transform);
  tf_broadcaster_.sendTransform(map_tf);
}

void RosDataProviderInterface::publishStaticTf(
    const gtsam::Pose3& pose,
    const std::string& parent_frame_id,
    const std::string& child_frame_id) {
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transform_stamped;
  // TODO(Toni): Warning: using ros::Time::now(), will that bring issues?
  static_transform_stamped.header.stamp = ros::Time::now();
  static_transform_stamped.header.frame_id = parent_frame_id;
  static_transform_stamped.child_frame_id = child_frame_id;
  poseToMsgTF(pose, &static_transform_stamped.transform);
  static_broadcaster.sendTransform(static_transform_stamped);
}

void RosDataProviderInterface::printParsedParams() const {
  static constexpr int kSeparatorWidth = 40;
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Left camera info:";
  vio_params_.camera_params_.at(0).print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Right camera info:";
  vio_params_.camera_params_.at(1).print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Frontend params:";
  vio_params_.frontend_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - IMU params:";
  vio_params_.imu_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=') << " - Backend params";
  vio_params_.backend_params_->print();
  LOG(INFO) << std::string(kSeparatorWidth, '=');
  vio_params_.lcd_params_.print();
  LOG(INFO) << std::string(kSeparatorWidth, '=');
}

void RosDataProviderInterface::msgTFtoPose(const geometry_msgs::Transform& tf,
                                           gtsam::Pose3* pose) {
  CHECK_NOTNULL(pose);

  *pose = gtsam::Pose3(
      gtsam::Rot3(gtsam::Quaternion(
          tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z)),
      gtsam::Point3(tf.translation.x, tf.translation.y, tf.translation.z));
}

void RosDataProviderInterface::poseToMsgTF(const gtsam::Pose3& pose,
                                           geometry_msgs::Transform* tf) {
  CHECK_NOTNULL(tf);

  tf->translation.x = pose.x();
  tf->translation.y = pose.y();
  tf->translation.z = pose.z();
  const gtsam::Quaternion& quat = pose.rotation().toQuaternion();
  tf->rotation.w = quat.w();
  tf->rotation.x = quat.x();
  tf->rotation.y = quat.y();
  tf->rotation.z = quat.z();
}

void RosDataProviderInterface::msgCamInfoToCameraParams(
    const sensor_msgs::CameraInfoConstPtr& cam_info,
    const std::string& cam_frame_id,
    VIO::CameraParams* cam_params) {
  CHECK_NOTNULL(cam_params);

  // Get intrinsics from incoming CameraInfo messages:
  cam_params->camera_id_ = cam_info->header.frame_id;
  CHECK(!cam_params->camera_id_.empty());

  cam_params->distortion_model_ = cam_info->distortion_model;
  CHECK(cam_params->distortion_model_ == "radtan" ||
        cam_params->distortion_model_ == "radial-tangential" ||
        cam_params->distortion_model_ == "equidistant");

  const std::vector<double>& distortion_coeffs = cam_info->D;
  CHECK_EQ(distortion_coeffs.size(), 4);
  VIO::CameraParams::convertDistortionVectorToMatrix(
      distortion_coeffs, &cam_params->distortion_coeff_);

  cam_params->image_size_ = cv::Size(cam_info->width, cam_info->height);

  cam_params->frame_rate_ = 0;  // TODO(marcus): is there a way to get this?

  std::array<double, 4> intrinsics = {
      cam_info->K[0], cam_info->K[4], cam_info->K[2], cam_info->K[5]};
  cam_params->intrinsics_ = intrinsics;
  VIO::CameraParams::convertIntrinsicsVectorToMatrix(cam_params->intrinsics_,
                                                     &cam_params->K_);

  VIO::CameraParams::createGtsamCalibration(cam_params->distortion_coeff_,
                                            cam_params->intrinsics_,
                                            &cam_params->calibration_);

  // Get extrinsics from the TF tree:
  tf2_ros::Buffer t_buffer;
  tf2_ros::TransformListener tf_listener(t_buffer);
  geometry_msgs::TransformStamped cam_tf;

  try {
    cam_tf = t_buffer.lookupTransform(base_link_frame_id_,
                                      cam_frame_id,
                                      ros::Time(0),
                                      ros::Duration(kTfLookupTimeout));
  } catch (tf2::TransformException& ex) {
    ROS_FATAL(
        "TF for left/right camera frames not available. Either publish to "
        "tree or provide CameraParameter yaml files.");
  }

  msgTFtoPose(cam_tf.transform, &cam_params->body_Pose_cam_);
}

}  // namespace VIO
