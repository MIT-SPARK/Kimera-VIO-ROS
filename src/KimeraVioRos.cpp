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
#include "kimera_vio_ros/RosBagDataProvider.h"
#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera_vio_ros/RosOnlineDataProvider.h"

int main(int argc, char* argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Initialize ROS node
  ros::init(argc, argv, "kimera_vio");
  ros::NodeHandle nh_("~");

  // Create dataset parser.
  VIO::RosDataProviderInterface::UniquePtr dataset_parser = nullptr;
  bool online_run = false;
  CHECK(nh_.getParam("online_run", online_run));
  if (online_run) {
    // Running ros online.
    dataset_parser = VIO::make_unique<VIO::RosOnlineDataProvider>();
  } else {
    // Parse rosbag.
    dataset_parser = VIO::make_unique<VIO::RosbagDataProvider>();
  }
  CHECK(dataset_parser);

  // Create actual VIO pipeline.
  VIO::Pipeline vio_pipeline(dataset_parser->pipeline_params_);

  // Register callback for inputs.
  dataset_parser->registerImuSingleCallback(
      std::bind(&VIO::Pipeline::fillSingleImuQueue,
                &vio_pipeline,
                std::placeholders::_1));

  dataset_parser->registerImuMultiCallback(
      std::bind(&VIO::Pipeline::fillMultiImuQueue,
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

  bool use_lcd = false;
  nh_.getParam("use_lcd", use_lcd);
  if (use_lcd) {
    vio_pipeline.registerLcdOutputCallback(
        std::bind(&VIO::RosDataProviderInterface::callbackLcdOutput,
                  std::ref(*CHECK_NOTNULL(dataset_parser.get())),
                  std::placeholders::_1));
  }

  // Spin dataset.
  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (dataset_parser->pipeline_params_.parallel_run_) {
    auto handle = std::async(std::launch::async,
                             &VIO::RosDataProviderInterface::spin,
                             std::move(dataset_parser));
    auto handle_pipeline = std::async(std::launch::async,
                                      &VIO::Pipeline::spin,
                                      &vio_pipeline);
    ros::start();
    // Run while ROS is ok and vio pipeline is not shutdown.
    // Ideally make a thread that shutdowns pipeline if ros is not ok.
    while (ros::ok()) {//&& vio_pipeline.spinViz()) {
      continue;
    }
    ROS_INFO("Shutting down ROS and VIO pipeline.");
    ros::shutdown();
    vio_pipeline.shutdown();
    handle_pipeline.get();
    is_pipeline_successful = !handle.get();
  } else {
    ros::start();
    while (ros::ok() && dataset_parser->spin()) {
      vio_pipeline.spin();
    }
    ROS_INFO("Shutting down ROS and VIO pipeline.");
    ros::shutdown();
    vio_pipeline.shutdown();
    is_pipeline_successful = true;
  }
  auto spin_duration = VIO::utils::Timer::toc(tic);
  ROS_WARN_STREAM("Spin took: " << spin_duration.count() << " ms.");
  ROS_INFO_STREAM("Pipeline successful? "
                  << (is_pipeline_successful ? "Yes!" : "No!"));
  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
