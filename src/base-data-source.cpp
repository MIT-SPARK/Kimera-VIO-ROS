/**
 * @file   base-data-source.cpp
 * @brief  Base class for ROS wrappers for spark vio.
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#include "spark-vio-ros/ros-data-source.h"

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>

namespace VIO {

RosBaseDataProvider::RosBaseDataProvider()
    : DataProvider(),
      it_(nullptr),
      stereo_calib_(),
      vio_output_(),
      nh_(),
      nh_private_("~"),
      vio_output_queue_("VIO output"),
      lcd_output_queue_("LCD output") {
  ROS_INFO(">>>>>>> Initializing Spark-VIO-ROS <<<<<<<");

  // Parse calibration info for camera and IMU
  // Calibration info on parameter server (Parsed from yaml)
  parseCameraData(&stereo_calib_);

  it_ = VIO::make_unique<image_transport::ImageTransport>(nh_);

  // Get ROS params
  ROS_ASSERT(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  ROS_ASSERT(nh_private_.getParam("world_frame_id", world_frame_id_));
  ROS_ASSERT(nh_private_.getParam("map_frame_id", map_frame_id_));

  // Publishers
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 10);
  frontend_stats_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("frontend_stats", 10);
  resiliency_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("resiliency", 10);
  imu_bias_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("imu_bias", 10);
  trajectory_pub_ = nh_.advertise<nav_msgs::Path>("optimized_trajectory", 10);
  posegraph_pub_ = nh_.advertise<pose_graph_tools::PoseGraph>("pose_graph", 10);
  pointcloud_pub_ =
      nh_.advertise<PointCloudXYZRGB>("time_horizon_pointcloud", 10);
  mesh_3d_frame_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("mesh", 5);
  debug_img_pub_ = it_->advertise("debug_mesh_img", 10);
}

RosBaseDataProvider::~RosBaseDataProvider() {}

cv::Mat RosBaseDataProvider::readRosImage(
    const sensor_msgs::ImageConstPtr& img_msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    LOG(WARNING) << "Converting image...";
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
  } else {
    CHECK_EQ(cv_ptr->encoding, sensor_msgs::image_encodings::MONO8)
        << "Expected image with MONO8 or BGR8 encoding.";
  }

  return cv_ptr->image;
}

cv::Mat RosBaseDataProvider::readRosDepthImage(
    const sensor_msgs::ImageConstPtr& img_msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr =
        cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  cv::Mat img_depth = cv_ptr->image;
  if (img_depth.type() != CV_16UC1) {
    LOG(WARNING) << "Converting img_depth.";
    img_depth.convertTo(img_depth, CV_16UC1);
  }
  return img_depth;
}

bool RosBaseDataProvider::parseCameraData(StereoCalibration* stereo_calib) {
  CHECK_NOTNULL(stereo_calib);
  // Parse camera calibration info (from param server)

  // Rate
  double rate;
  nh_private_.getParam("camera_rate_hz", rate);

  // Resoltuion
  std::vector<int> resolution;
  CHECK(nh_private_.getParam("camera_resolution", resolution));
  CHECK_EQ(resolution.size(), 2);

  // Get distortion/intrinsics/extrinsics for each camera
  for (int i = 0; i < 2; i++) {
    std::string camera_name;
    CameraParams camera_param_i;
    // Fill in rate and resolution
    camera_param_i.image_size_ = cv::Size(resolution[0], resolution[1]);
    // Terminology wrong but following rest of the repo
    camera_param_i.frame_rate_ = 1.0 / rate;

    if (i == 0) {
      camera_name = "left_camera_";
    } else {
      camera_name = "right_camera_";
    }
    // Parse intrinsics (camera matrix)
    std::vector<double> intrinsics;
    nh_private_.getParam(camera_name + "intrinsics", intrinsics);
    CHECK_EQ(intrinsics.size(), 4u);
    camera_param_i.intrinsics_ = intrinsics;
    // Conver intrinsics to camera matrix (OpenCV format)
    camera_param_i.camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_param_i.camera_matrix_.at<double>(0, 0) = intrinsics[0];
    camera_param_i.camera_matrix_.at<double>(1, 1) = intrinsics[1];
    camera_param_i.camera_matrix_.at<double>(0, 2) = intrinsics[2];
    camera_param_i.camera_matrix_.at<double>(1, 2) = intrinsics[3];

    // Parse extrinsics (rotation and translation)
    std::vector<double> extrinsics;
    // Encode calibration frame to body frame
    std::vector<double> frame_change;
    CHECK(nh_private_.getParam(camera_name + "extrinsics", extrinsics));
    CHECK(nh_private_.getParam("calibration_to_body_frame", frame_change));
    CHECK_EQ(extrinsics.size(), 16u);
    CHECK_EQ(frame_change.size(), 16u);
    // Place into matrix
    // 4 4 is hardcoded here because currently only accept extrinsic input
    // in homoegeneous format [R T ; 0 1]
    cv::Mat E_calib = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat calib2body = cv::Mat::zeros(4, 4, CV_64F);
    for (int k = 0; k < 16; k++) {
      int row = k / 4;  // Integer division, truncation of fractional part.
      int col = k % 4;
      E_calib.at<double>(row, col) = extrinsics[k];
      calib2body.at<double>(row, col) = frame_change[k];
    }

    // TODO(Yun): Check frames convention!
    // Extrinsics in body frame
    cv::Mat E_body = calib2body * E_calib;

    // restore back to vector form
    std::vector<double> extrinsics_body;
    for (int k = 0; k < 16; k++) {
      int row = k / 4;  // Integer division, truncation of fractional part.
      int col = k % 4;
      extrinsics_body.push_back(E_body.at<double>(row, col));
    }

    camera_param_i.body_Pose_cam_ =
        UtilsOpenCV::Vec2pose(extrinsics_body, 4, 4);

    // Distortion model
    std::string distortion_model;
    nh_private_.getParam("distortion_model", distortion_model);
    camera_param_i.distortion_model_ = distortion_model;

    // Parse distortion
    std::vector<double> d_coeff;
    nh_private_.getParam(camera_name + "distortion_coefficients", d_coeff);
    cv::Mat distortion_coeff;

    switch (d_coeff.size()) {
      // if given 4 coefficients
      case (4):
        ROS_INFO(
            "Using radtan or equidistant model (4 coefficients) for camera %d",
            i);
        distortion_coeff = cv::Mat::zeros(1, 4, CV_64F);
        distortion_coeff.at<double>(0, 0) = d_coeff[0];  // k1
        distortion_coeff.at<double>(0, 1) = d_coeff[1];  // k2
        distortion_coeff.at<double>(0, 3) = d_coeff[2];  // p1 or k3
        distortion_coeff.at<double>(0, 4) = d_coeff[3];  // p2 or k4
        break;

      case (5):  // if given 5 coefficients
        ROS_INFO("Using radtan model (5 coefficients) for camera %d", i);
        distortion_coeff = cv::Mat::zeros(1, 5, CV_64F);
        for (int k = 0; k < 5; k++) {
          distortion_coeff.at<double>(0, k) = d_coeff[k];  // k1, k2, k3, p1, p2
        }
        break;

      default:  // otherwise
        ROS_FATAL("Unsupported distortion format.");
    }

    camera_param_i.distortion_coeff_ = distortion_coeff;

    // TODO(unknown): add skew (can add switch statement when parsing
    // intrinsics)
    camera_param_i.calibration_ =
        gtsam::Cal3DS2(intrinsics[0],                       // fx
                       intrinsics[1],                       // fy
                       0.0,                                 // skew
                       intrinsics[2],                       // u0
                       intrinsics[3],                       // v0
                       distortion_coeff.at<double>(0, 0),   //  k1
                       distortion_coeff.at<double>(0, 1),   //  k2
                       distortion_coeff.at<double>(0, 3),   //  p1
                       distortion_coeff.at<double>(0, 4));  //  p2

    if (i == 0) {
      stereo_calib->left_camera_info_ = camera_param_i;
    } else {
      stereo_calib->right_camera_info_ = camera_param_i;
    }
  }

  // Calculate the pose of right camera relative to the left camera
  stereo_calib->camL_Pose_camR_ =
      (stereo_calib->left_camera_info_.body_Pose_cam_)
          .between(stereo_calib->right_camera_info_.body_Pose_cam_);

  ROS_INFO("Parsed stereo camera calibration");
  return true;
}

bool RosBaseDataProvider::parseImuData(ImuData* imudata, ImuParams* imuparams) {
  // Parse IMU calibration info (from param server)
  double rate, rate_std, rate_maxMismatch, gyro_noise, gyro_walk, acc_noise,
      acc_walk, imu_shift;

  std::vector<double> extrinsics;

  ROS_ASSERT(nh_private_.getParam("imu_rate_hz", rate));
  ROS_ASSERT(nh_private_.getParam("gyroscope_noise_density", gyro_noise));
  ROS_ASSERT(nh_private_.getParam("gyroscope_random_walk", gyro_walk));
  ROS_ASSERT(nh_private_.getParam("accelerometer_noise_density", acc_noise));
  ROS_ASSERT(nh_private_.getParam("accelerometer_random_walk", acc_walk));
  ROS_ASSERT(nh_private_.getParam("imu_extrinsics", extrinsics));
  ROS_ASSERT(nh_private_.getParam("imu_shift", imu_shift));

  // TODO(Sandro): Do we need these parameters??
  imudata->nominal_imu_rate_ = 1.0 / rate;
  imudata->imu_rate_ = 1.0 / rate;
  imudata->imu_rate_std_ = 0.00500009;          // set to 0 for now
  imudata->imu_rate_maxMismatch_ = 0.00500019;  // set to 0 for now

  // Gyroscope and accelerometer noise parameters
  imuparams->gyro_noise_ = gyro_noise;
  imuparams->gyro_walk_ = gyro_walk;
  imuparams->acc_noise_ = acc_noise;
  imuparams->acc_walk_ = acc_walk;
  imuparams->imu_shift_ =
      imu_shift;  // Defined as t_imu = t_cam + imu_shift (see: Kalibr)

  ROS_INFO("Parsed IMU calibration");
  return true;
}

void RosBaseDataProvider::publishOutput(const SpinOutputPacket& vio_output) {
  publishTf(vio_output);
  if (odometry_pub_.getNumSubscribers() > 0) {
    publishState(vio_output);
  }
  // Publish 3d mesh (not the time-horizon one! just the per-frame one)
  if (mesh_3d_frame_pub_.getNumSubscribers() > 0) {
    publishPerFrameMesh3D(vio_output);
  }
  // Publish 2d mesh debug image
  if (debug_img_pub_.getNumSubscribers() > 0) {
    publishDebugImage(vio_output.getTimestamp(), vio_output.mesh_2d_img_);
  }
  if (pointcloud_pub_.getNumSubscribers() > 0) {
    publishTimeHorizonPointCloud(vio_output.getTimestamp(),
                                 vio_output.points_with_id_VIO_,
                                 vio_output.lmk_id_to_lmk_type_map_);
  }
  if (frontend_stats_pub_.getNumSubscribers() > 0) {
    publishFrontendStats(vio_output);
  }
  // Publish Resiliency
  if (resiliency_pub_.getNumSubscribers() > 0) {
    publishResiliency(vio_output);
  }
  if (imu_bias_pub_.getNumSubscribers() > 0) {
    publishImuBias(vio_output);
  }
}

void RosBaseDataProvider::publishLCDOutput(const LoopClosureDetectorOutputPayload& lcd_output) {
  publishTf(lcd_output);
  if (trajectory_pub_.getNumSubscribers() > 0 ) {
    publishOptimizedTrajectory(lcd_output);
  }
}

void RosBaseDataProvider::publishTimeHorizonPointCloud(
    const Timestamp& timestamp, const PointsWithIdMap& points_with_id,
    const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map) const {
  PointCloudXYZRGB::Ptr msg(new PointCloudXYZRGB);
  msg->header.frame_id = world_frame_id_;
  msg->is_dense = true;
  msg->height = 1;
  msg->width = points_with_id.size();
  msg->points.resize(points_with_id.size());

  LOG(ERROR) << "Points with id size: " << msg->points.size();

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

void RosBaseDataProvider::publishDebugImage(const Timestamp& timestamp,
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
//    const SpinOutputPacket& vio_output) {
//  const Mesh3D& mesh_3d = vio_output.mesh_3d_;
//  size_t number_mesh_3d_polygons = mesh_3d.getNumberOfPolygons();
//}

void RosBaseDataProvider::publishPerFrameMesh3D(
    const SpinOutputPacket& vio_output) const {
  const Mesh2D& mesh_2d = vio_output.mesh_2d_;
  const Mesh3D& mesh_3d = vio_output.mesh_3d_;
  size_t number_mesh_2d_polygons = mesh_2d.getNumberOfPolygons();
  size_t mesh_2d_poly_dim = mesh_2d.getMeshPolygonDimension();

  static const size_t cam_width =
      stereo_calib_.left_camera_info_.image_size_.width;
  static const size_t cam_height =
      stereo_calib_.left_camera_info_.image_size_.height;
  DCHECK_GT(cam_width, 0);
  DCHECK_GT(cam_height, 0);

  pcl_msgs::PolygonMesh::Ptr msg(new pcl_msgs::PolygonMesh());
  msg->header.stamp.fromNSec(vio_output.getTimestamp());
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

void RosBaseDataProvider::publishState(
    const SpinOutputPacket& vio_output) const {
  // Get latest estimates for odometry.
  const gtsam::Pose3& pose = vio_output.getEstimatedPose();
  const gtsam::Vector3& velocity = vio_output.getEstimatedVelocity();
  const Timestamp& ts = vio_output.getTimestamp();
  const gtsam::Matrix6& pose_cov = vio_output.getEstimatedPoseCov();
  const gtsam::Matrix3& vel_cov = vio_output.getEstimatedVelCov();

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
  const gtsam::Rot3& rotation = pose.rotation();
  const gtsam::Quaternion& quaternion = rotation.toQuaternion();
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

void RosBaseDataProvider::publishTf(const SpinOutputPacket& vio_output) {
  const Timestamp& ts = vio_output.getTimestamp();
  const gtsam::Pose3& pose = vio_output.getEstimatedPose();
  const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();
  // Publish base_link TF.
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp.fromNSec(ts);
  odom_tf.header.frame_id = world_frame_id_;
  odom_tf.child_frame_id = base_link_frame_id_;

  odom_tf.transform.translation.x = pose.x();
  odom_tf.transform.translation.y = pose.y();
  odom_tf.transform.translation.z = pose.z();
  odom_tf.transform.rotation.w = quaternion.w();
  odom_tf.transform.rotation.x = quaternion.x();
  odom_tf.transform.rotation.y = quaternion.y();
  odom_tf.transform.rotation.z = quaternion.z();
  tf_broadcaster_.sendTransform(odom_tf);
}

void RosBaseDataProvider::publishFrontendStats(
    const SpinOutputPacket& vio_output) const {
  // Get frontend data for resiliency output
  const DebugTrackerInfo& debug_tracker_info = vio_output.getTrackerInfo();

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

void RosBaseDataProvider::publishResiliency(
    const SpinOutputPacket& vio_output) const {
  // Get frontend and velocity covariance data for resiliency output
  const DebugTrackerInfo& debug_tracker_info = vio_output.getTrackerInfo();
  const gtsam::Matrix3& vel_cov = vio_output.getEstimatedVelCov();
  const gtsam::Matrix6& pose_cov = vio_output.getEstimatedPoseCov();

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

void RosBaseDataProvider::publishImuBias(
    const SpinOutputPacket& vio_output) const {
  // Get imu bias to output
  const ImuBias& imu_bias = vio_output.getEstimatedBias();
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

void RosBaseDataProvider::publishOptimizedTrajectory(
    const LoopClosureDetectorOutputPayload& lcd_output) const {
  // Get pgo-optimized trajectory
  const Timestamp& ts = lcd_output.timestamp_kf_;
  const gtsam::Values& trajectory = lcd_output.states_;
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

pose_graph_tools::PoseGraph RosBaseDataProvider::GtsamToPosegraphMsg(
    const gtsam::NonlinearFactorGraph& nfg,
    const gtsam::Values& values) {
  // pose graph getter
  pose_graph_tools::PoseGraph pose_graph;

  // first store the factors as edges
  for (size_t i = 0; i < nfg.size(); i++) {
    // check if between factor
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(nfg[i])) {
      // convert to between factor
      gtsam::BetweenFactor<gtsam::Pose3> factor =
            *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(nfg[i]);
      // convert between factor to PoseGraphEdge type
      pose_graph_tools::PoseGraphEdge edge;
      edge.header.frame_id = world_frame_id_;
      edge.key_from = factor.front();
      edge.key_to = factor.back();
      if (edge.key_to == edge.key_from + 1) { // check if odom
        edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
      } else {
        edge.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
      }
      // transforms - translation
      edge.pose.position.x = factor.measured().translation().x();
      edge.pose.position.y = factor.measured().translation().y();
      edge.pose.position.z = factor.measured().translation().z();
      // transforms - rotation (to quaternion)
      edge.pose.orientation.x = factor.measured().rotation().toQuaternion().x();
      edge.pose.orientation.y = factor.measured().rotation().toQuaternion().y();
      edge.pose.orientation.z = factor.measured().rotation().toQuaternion().z();
      edge.pose.orientation.w = factor.measured().rotation().toQuaternion().w();

      // TODO: add covariance
      pose_graph.edges.push_back(edge);
    }
  }

  // Then store the values as nodes
  gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    pose_graph_tools::PoseGraphNode node;
    node.key = key_list[i];
    // pose - translation
    node.pose.position.x = values.at<gtsam::Pose3>(i).translation().x();
    node.pose.position.y = values.at<gtsam::Pose3>(i).translation().y();
    node.pose.position.z = values.at<gtsam::Pose3>(i).translation().z();
    // pose - rotation (to quaternion)
    node.pose.orientation.x = values.at<gtsam::Pose3>(i).rotation().toQuaternion().x();
    node.pose.orientation.y = values.at<gtsam::Pose3>(i).rotation().toQuaternion().y();
    node.pose.orientation.z = values.at<gtsam::Pose3>(i).rotation().toQuaternion().z();
    node.pose.orientation.w = values.at<gtsam::Pose3>(i).rotation().toQuaternion().w();

    pose_graph.nodes.push_back(node);
  }

  return pose_graph;
}

void RosBaseDataProvider::publishPoseGraph(
    const LoopClosureDetectorOutputPayload& lcd_output) { 
  // Get the factor graph 
  const Timestamp& ts = lcd_output.timestamp_kf_;
  const gtsam::NonlinearFactorGraph& nfg = lcd_output.nfg_;
  const gtsam::Values& values = lcd_output.states_;

  pose_graph_tools::PoseGraph graph = GtsamToPosegraphMsg(nfg, values);
  graph.header.stamp.fromNSec(ts);
  graph.header.frame_id = world_frame_id_;
  posegraph_pub_.publish(graph);
}

void RosBaseDataProvider::publishTf(
    const LoopClosureDetectorOutputPayload& lcd_output) {
  const Timestamp& ts = lcd_output.timestamp_kf_;
  const gtsam::Pose3& w_Pose_map = lcd_output.W_Pose_Map_;
  const gtsam::Quaternion& w_Quat_map = w_Pose_map.rotation().toQuaternion();
  // Publish map TF.
  geometry_msgs::TransformStamped map_tf;
  map_tf.header.stamp.fromNSec(ts);
  map_tf.header.frame_id = world_frame_id_;
  map_tf.child_frame_id = map_frame_id_;

  map_tf.transform.translation.x = w_Pose_map.x();
  map_tf.transform.translation.y = w_Pose_map.y();
  map_tf.transform.translation.z = w_Pose_map.z();
  map_tf.transform.rotation.w = w_Quat_map.w();
  map_tf.transform.rotation.x = w_Quat_map.x();
  map_tf.transform.rotation.y = w_Quat_map.y();
  map_tf.transform.rotation.z = w_Quat_map.z();
  tf_broadcaster_.sendTransform(map_tf);
}

// VIO output callback at keyframe rate
void RosBaseDataProvider::callbackKeyframeRateVioOutput(
    const SpinOutputPacket& vio_output) {
  // The code here should be lighting fast or we will be blocking the backend
  // thread in the VIO. This is actually running in the backend thread, as
  // such do not modify things other than thread-safe stuff.
  vio_output_queue_.push(vio_output);
}

void RosBaseDataProvider::callbackLoopClosureOutput(
    const LoopClosureDetectorOutputPayload& lcd_output) {
  lcd_output_queue_.push(lcd_output);
}

}  // namespace VIO
