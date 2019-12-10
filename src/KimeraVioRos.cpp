/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Spark-VIO
 * @author Antoni Rosinol
 */

#include <future>

// Still need gflags for parameters in VIO
#include <gflags/gflags.h>
#include <glog/logging.h>

// Dependencies from ROS
#include <ros/ros.h>

// Dependencies from VIO
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/utils/Timer.h>

// Dependencies from this repository
#include "kimera_ros/RosDataProviderInterface.h"
#include "kimera_ros/RosOnlineDataProvider.h"
#include "kimera_ros/RosBagDataProvider.h"

DEFINE_bool(parallel_run, true, "Run VIO parallel or sequential");
DEFINE_bool(online_run, true, "RUN VIO ROS online or offline");

int main(int argc, char* argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Initialize ROS node
  ros::init(argc, argv, "kimera_vio");

  // Create dataset parser.
  // TODO(marcus): make unique_ptr
  VIO::RosDataProviderInterface::Ptr dataset_parser;
  if (FLAGS_online_run) {
    // Running ros online.
    dataset_parser = std::make_shared<VIO::RosOnlineDataProvider>();
  } else {
    // Parse rosbag.
    dataset_parser = std::make_shared<VIO::RosbagDataProvider>();
  }

  // Create actual VIO pipeline.
  VIO::Pipeline vio_pipeline(dataset_parser->pipeline_params_);

  // Register callback for inputs.
  dataset_parser->registerImuSingleCallback(
      std::bind(&VIO::Pipeline::fillSingleImuQueue,
                &vio_pipeline,
                std::placeholders::_1));

  dataset_parser->registerLeftFrameCallback(
      std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                &vio_pipeline,
                std::placeholders::_1));

  dataset_parser->registerRightFrameCallback(
      std::bind(&VIO::Pipeline::fillRightFrameQueue,
                &vio_pipeline,
                std::placeholders::_1));

  // Register callback to retrieve vio pipeline output from all modules.
  vio_pipeline.registerBackendOutputCallback(
      std::bind(&VIO::RosDataProviderInterface::callbackBackendOutput,
                std::ref(*CHECK_NOTNULL(dataset_parser.get())),
                std::placeholders::_1));

  vio_pipeline.registerFrontendOutputCallback(
      std::bind(&VIO::RosDataProviderInterface::callbackFrontendOutput,
                std::ref(*CHECK_NOTNULL(dataset_parser.get())),
                std::placeholders::_1));

  vio_pipeline.registerMesherOutputCallback(
      std::bind(&VIO::RosDataProviderInterface::callbackMesherOutput,
                std::ref(*CHECK_NOTNULL(dataset_parser.get())),
                std::placeholders::_1));

  // TODO(marcus): only register this if we have `use_lcd` enabled.
  bool use_lcd;
  ros::param::get("use_lcd", use_lcd);
  if (use_lcd) {
    vio_pipeline.registerLcdOutputCallback(
        std::bind(&VIO::RosDataProviderInterface::callbackLcdOutput,
                  std::ref(*CHECK_NOTNULL(dataset_parser.get())),
                  std::placeholders::_1));
  }

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (FLAGS_parallel_run) {
    auto handle = std::async(
        std::launch::async, &VIO::RosDataProviderInterface::spin, dataset_parser);
    ros::start();
    // Run while ROS is ok and vio pipeline is not shutdown.
    // Ideally make a thread that shutdowns pipeline if ros is not ok.
    while (ros::ok() && vio_pipeline.spinViz()) {
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
