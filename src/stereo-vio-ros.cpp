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
  std::unique_ptr<VIO::RosBaseDataProvider> dataset_parser;
  if (FLAGS_online_run) {
    // Running ros online.
    dataset_parser = VIO::make_unique<VIO::RosDataProvider>();
  } else {
    // Parse rosbag.
    dataset_parser = VIO::make_unique<VIO::RosbagDataProvider>();
  }

  // Create actual VIO pipeline.
  VIO::Pipeline vio_pipeline(dataset_parser->getParams(), FLAGS_parallel_run);

  // Register callback to vio_pipeline.
  dataset_parser->registerVioCallback(
      std::bind(&VIO::Pipeline::spin, &vio_pipeline, std::placeholders::_1));

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (FLAGS_parallel_run) {
    auto handle =
        std::async(std::launch::async, &VIO::RosBaseDataProvider::spin,
                   std::move(dataset_parser));
    auto handle_pipeline =
        std::async(std::launch::async, &VIO::Pipeline::shutdownWhenFinished,
                   &vio_pipeline);
    ros::start();
    while (ros::ok()) {
      vio_pipeline.spinViz(false);
    }
    ros::shutdown();
    vio_pipeline.shutdown();
    is_pipeline_successful = handle.get();
    handle_pipeline.get();
  } else {
    is_pipeline_successful = dataset_parser->spin();
  }
  auto spin_duration = VIO::utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";
  LOG(INFO) << "Pipeline successful? "
            << (is_pipeline_successful ? "Yes!" : "No!");
  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
