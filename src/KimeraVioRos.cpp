/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Kimera-VIO
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera_vio_ros/KimeraVioRos.h"

#include <future>

// Still need gflags for parameters in VIO
#include <gflags/gflags.h>
#include <glog/logging.h>

// Dependencies from ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

// Dependencies from VIO
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/utils/Timer.h>

// Dependencies from this repository
#include "kimera_vio_ros/RosBagDataProvider.h"
#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera_vio_ros/RosOnlineDataProvider.h"

namespace VIO {

KimeraVioRos::KimeraVioRos()
    : vio_params_(nullptr),
      vio_pipeline_(nullptr),
      data_provider_(nullptr),
      ros_display_(nullptr),
      restart_vio_pipeline_srv_(),
      restart_vio_pipeline_(false) {
  // Add rosservice to restart VIO pipeline if requested.
  ros::NodeHandle nh_("~");
  restart_vio_pipeline_srv_ = nh_.advertiseService(
      "restart_kimera_vio", &KimeraVioRos::restartKimeraVio, this);
  std::string params_folder_path;
  CHECK(nh_.getParam("params_folder_path", params_folder_path));
  CHECK(!params_folder_path.empty());
  vio_params_ = std::make_shared<VioParams>(params_folder_path);
}

bool KimeraVioRos::runKimeraVio() {
  // First, destroy VIO pipeline, this will in turn call the shutdown of
  // the data provider.
  // NOTE: had the data provider been destroyed before, the vio would be calling
  // the shutdown function of a deleted object, aka segfault.
  VLOG(1) << "Destroy Ros Display.";
  ros_display_.reset();

  VLOG(1) << "Creating Ros Display.";
  CHECK(vio_params_);
  ros_display_ = VIO::make_unique<RosDisplay>(*vio_params_);

  VLOG(1) << "Destroy Vio Pipeline.";
  vio_pipeline_.reset();

  // Then, create Kimera-VIO from scratch.
  VLOG(1) << "Creating Kimera-VIO.";
  CHECK(ros_display_);
  vio_pipeline_ = VIO::make_unique<VIO::Pipeline>(*vio_params_,
                                                  std::move(ros_display_));
  CHECK(vio_pipeline_) << "Vio pipeline construction failed.";

  // Second, destroy dataset parser.
  VLOG(1) << "Destroy Data Provider.";
  data_provider_.reset();

  // Then, create dataset parser.
  VLOG(1) << "Creating Data Provider.";
  data_provider_ = createDataProvider(*vio_params_);
  CHECK(data_provider_) << "Data provider construction failed.";

  // Finally, connect data_provider and vio_pipeline
  VLOG(1) << "Connecting Vio Pipeline and Data Provider.";
  connectVIO();

  // Run
  return spin();
}

bool KimeraVioRos::spin() {
  CHECK(vio_params_);
  CHECK(vio_pipeline_);
  CHECK(data_provider_);

  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (vio_params_->parallel_run_) {
    std::future<bool> vio_viz_handle =
        std::async(std::launch::async,
                   &VIO::Pipeline::spinViz,
                   vio_pipeline_.get());
    std::future<bool> vio_pipeline_handle = std::async(
        std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
    // Run while ROS is ok and vio pipeline is not shutdown.
    // Ideally make a thread that shutdowns pipeline if ros is not ok.
    ros::Rate rate(20);  // Check pipeline status at 20Hz
    while (ros::ok() && !restart_vio_pipeline_) {
      // Print stats at 10hz
      LOG_EVERY_N(INFO, 10) << vio_pipeline_->printStatistics();
      rate.sleep();
    }
    if (!restart_vio_pipeline_) {
      LOG(INFO) << "Shutting down ROS and Kimera-VIO.";
      ros::shutdown();
    } else {
      LOG(INFO) << "Restarting Kimera-VIO.";
    }
    // TODO(Toni): right now vio shutsdown data provider, maybe we should
    // explicitly shutdown data provider: data_provider_->shutdown();
    vio_pipeline_->shutdown();
    LOG(INFO) << "Joining Kimera-VIO thread.";
    vio_pipeline_handle.get();
    LOG(INFO) << "Kimera-VIO thread joined successfully.";
    LOG(INFO) << "Joining RosDisplay thread.";
    is_pipeline_successful = !vio_viz_handle.get();
    LOG(INFO) << "RosDisplay thread joined successfully.";
    if (restart_vio_pipeline_) {
      // Mind that this is a recursive call! As we call this function
      // inside runKimeraVio. Sorry, couldn't find a better way.
      restart_vio_pipeline_ = false;
      LOG(INFO) << "Restarting...";
      return runKimeraVio();
    }
  } else {
    ros::start();
    while (ros::ok() && data_provider_->spin() && vio_pipeline_->spin()) {
      LOG(INFO) << vio_pipeline_->printStatistics();
      vio_pipeline_->spinViz();
    }
    LOG(INFO) << "Shutting down ROS and VIO pipeline.";
    ros::shutdown();
    vio_pipeline_->shutdown();
    is_pipeline_successful = true;
  }
  auto spin_duration = VIO::utils::Timer::toc(tic);
  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";
  LOG(INFO) << "Pipeline successful? "
            << (is_pipeline_successful ? "Yes!" : "No!");
  return is_pipeline_successful;
}

RosDataProviderInterface::UniquePtr KimeraVioRos::createDataProvider(
    const VioParams& vio_params) {
  ros::NodeHandle nh_("~");
  bool online_run = false;
  CHECK(nh_.getParam("online_run", online_run));
  if (online_run) {
    // Running ros online.
    return VIO::make_unique<RosOnlineDataProvider>(vio_params);
  } else {
    // Parse rosbag.
    return VIO::make_unique<RosbagDataProvider>(vio_params);
  }
  return nullptr;
}

void KimeraVioRos::connectVIO() {
  CHECK(data_provider_);
  CHECK(vio_pipeline_);

  // Register VIO pipeline callbacks
  // Register callback to shutdown data provider in case VIO pipeline
  // shutsdown.
  vio_pipeline_->registerShutdownCallback(std::bind(
      &VIO::DataProviderInterface::shutdown, std::ref(*data_provider_)));

  // Register callback to retrieve vio pipeline output from all modules.
  vio_pipeline_->registerBackendOutputCallback(
      std::bind(&VIO::RosDisplay::callbackBackendOutput,
                std::ref(*ros_display_),
                std::placeholders::_1));

  vio_pipeline_->registerFrontendOutputCallback(
      std::bind(&VIO::RosDisplay::callbackFrontendOutput,
                std::ref(*ros_display_),
                std::placeholders::_1));

  vio_pipeline_->registerMesherOutputCallback(
      std::bind(&VIO::RosDisplay::callbackMesherOutput,
                std::ref(*ros_display_),
                std::placeholders::_1));

  bool use_lcd = false;
  ros::NodeHandle nh_("~");
  nh_.getParam("use_lcd", use_lcd);
  if (use_lcd) {
    vio_pipeline_->registerLcdOutputCallback(
        std::bind(&VIO::RosDisplay::callbackLcdOutput,
                  std::ref(*ros_display_),
                  std::placeholders::_1));
  }

  // Register Data Provider callbacks
  data_provider_->registerImuSingleCallback(
      std::bind(&VIO::Pipeline::fillSingleImuQueue,
                std::ref(*vio_pipeline_),
                std::placeholders::_1));

  data_provider_->registerImuMultiCallback(
      std::bind(&VIO::Pipeline::fillMultiImuQueue,
                std::ref(*vio_pipeline_),
                std::placeholders::_1));

  data_provider_->registerLeftFrameCallback(
      std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                std::ref(*vio_pipeline_),
                std::placeholders::_1));

  data_provider_->registerRightFrameCallback(
      std::bind(&VIO::Pipeline::fillRightFrameQueue,
                std::ref(*vio_pipeline_),
                std::placeholders::_1));
}

bool KimeraVioRos::restartKimeraVio(std_srvs::Trigger::Request& request,
                                    std_srvs::Trigger::Response& response) {
  if (!restart_vio_pipeline_) {
    restart_vio_pipeline_ = true;
    response.message = "Kimera-VIO restart requested.";
    response.success = true;
  } else {
    response.message = "Kimera-VIO should already be restarting...";
    response.success = false;
  }
  LOG(WARNING) << response.message;
  return true;
}

}  // namespace VIO
