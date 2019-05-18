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

////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample using ROS wrapper example
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "spark_vio");

  // Initialize Google's flags library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Parse topic names from parameter server
  ros::NodeHandle nh; 
  std::string left_camera_topic, right_camera_topic, imu_topic; 
  nh.getParam("left_camera_topic", left_camera_topic);
  nh.getParam("right_camera_topic", right_camera_topic); 
  nh.getParam("imu_topic", imu_topic); 

  VIO::ETHDatasetParser eth_dataset_parser; // Dummy ETH data (Since need this in pipeline)
  VIO::RosbagDataProvider rosbag_parser(left_camera_topic, right_camera_topic, imu_topic, FLAGS_rosbag_path);

  VIO::Pipeline vio_pipeline (&eth_dataset_parser, rosbag_parser.getImuParams(), false); // run sequential

  // Register callback to vio_pipeline.
  rosbag_parser.registerVioCallback(
      std::bind(&VIO::Pipeline::spin, &vio_pipeline, std::placeholders::_1)); 

  // Spin dataset and handle threads
  auto tic = VIO::utils::Timer::tic();

  bool is_pipeline_successful = false;
  is_pipeline_successful = rosbag_parser.spin();

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
