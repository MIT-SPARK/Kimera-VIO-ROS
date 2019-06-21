/* @file   stereoVIOROS.cpp
 * @brief  ROS Wrapper for Spark-VIO
 * @author Yun Chang based off stereoVIOEuroc.cpp(in spark-VIO repo)
 */

#include <future>

// Still need gflags for parameters in VIO
#include <gflags/gflags.h>
#include <glog/logging.h>

// Dependencies from ROS 
#include <ros/ros.h>

// Dependencies from VIO
#include <utils/Timer.h>
#include <LoggerMatlab.h>

// Dependencies from this repository 
#include "RosbagDataSource.h"

DEFINE_string(rosbag_path, "rosbag", "Path to rosbag");
DEFINE_string(left_camera_topic, "/cam0/image_raw", "Left camera topic name");
DEFINE_string(right_camera_topic, "/cam1/image_raw", "Right camera topic name");
DEFINE_string(imu_topic, "/imu0", "IMU topic name");

////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample using ROS wrapper example
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "spark_vio");

  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Parse topic names from parameter server
  ros::NodeHandle nh; 

  VIO::ETHDatasetParser eth_dataset_parser; // Dummy ETH data (Since need this in pipeline)
  VIO::RosbagDataProvider rosbag_parser(FLAGS_left_camera_topic, FLAGS_right_camera_topic, 
                      FLAGS_imu_topic, FLAGS_rosbag_path);

  VIO::Pipeline vio_pipeline (&eth_dataset_parser, rosbag_parser.getImuParams());

  // Register callback to vio_pipeline.
  rosbag_parser.registerVioCallback(
      std::bind(&VIO::Pipeline::spin, &vio_pipeline, std::placeholders::_1)); 

  // Spin dataset and handle threads
  auto tic = VIO::utils::Timer::tic();
  auto handle = std::async(std::launch::async,
                           &VIO::RosbagDataProvider::spin, &rosbag_parser);
  auto handle_pipeline = std::async(std::launch::async,
                         &VIO::Pipeline::shutdownWhenFinished, &vio_pipeline);
  vio_pipeline.spinViz();
  const bool is_pipeline_successful = handle.get();
  handle_pipeline.get();

  auto spin_duration = VIO::utils::Timer::toc(tic);

  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";

  if (is_pipeline_successful) {
    // Log overall time of pipeline run.
    VIO::LoggerMatlab logger;
    logger.openLogFiles(11);
    logger.logPipelineOverallTiming(spin_duration);
    logger.closeLogFiles();
  }

  return is_pipeline_successful? EXIT_SUCCESS : EXIT_FAILURE;
}
