/* @file   stereo-vio-ros.cpp
 * @brief  ROS Wrapper for Spark-VIO
 * @author Yun Chang
 * @author Antoni Rosinol
 */

#include <future>

// Still need gflags for parameters in VIO
#include <gflags/gflags.h>
#include <glog/logging.h>

// Dependencies from ROS
#include <ros/ros.h>

// Dependencies from VIO
#include <LoggerMatlab.h>
#include <utils/Timer.h>

// Dependencies from this repository
#include "spark-vio-ros/ros-base-data-source.h"
#include "spark-vio-ros/ros-data-source.h"
#include "spark-vio-ros/rosbag-data-source.h"

DEFINE_bool(parallel_run, true, "Run VIO parallel or sequential");
DEFINE_bool(online_run, true, "RUN VIO ROS online or offline");

////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample using ROS wrapper example
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Initialize ROS node
  ros::init(argc, argv, "spark_vio");

  // Create dataset parser.
  std::shared_ptr<VIO::RosBaseDataProvider> dataset_parser;
  if (FLAGS_online_run) {
    // Running ros online.
    dataset_parser = std::make_shared<VIO::RosDataProvider>();
  } else {
    // Parse rosbag.
    dataset_parser = std::make_shared<VIO::RosbagDataProvider>();
  }

  // Create actual VIO pipeline.
  VIO::Pipeline vio_pipeline(dataset_parser->getParams(), FLAGS_parallel_run);

  // Register callback to vio_pipeline.
  dataset_parser->registerVioCallback(
      std::bind(&VIO::Pipeline::spin, &vio_pipeline, std::placeholders::_1));

  // Register callback to retrieve vio pipeline output.
  vio_pipeline.registerKeyFrameRateOutputCallback(
      std::bind(&VIO::RosBaseDataProvider::callbackKeyframeRateVioOutput,
                dataset_parser, std::placeholders::_1));

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (FLAGS_parallel_run) {
    auto handle = std::async(std::launch::async,
                             &VIO::RosBaseDataProvider::spin, dataset_parser);
    ros::start();
    // Run while ROS is ok and vio pipeline is not shutdown.
    // Ideally make a thread that shutdowns pipeline if ros is not ok.
    while (ros::ok() && vio_pipeline.spinViz(false)) {
      continue;
    };
    ROS_INFO("Shutting down ROS and VIO pipeline.");
    ros::shutdown();
    vio_pipeline.shutdown();
    is_pipeline_successful = handle.get();
  } else {
    is_pipeline_successful = dataset_parser->spin();
  }
  auto spin_duration = VIO::utils::Timer::toc(tic);
  ROS_WARN_STREAM("Spin took: " << spin_duration.count() << " ms.");
  ROS_INFO_STREAM("Pipeline successful? "
                  << (is_pipeline_successful ? "Yes!" : "No!"));
  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
