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
#include <kimera-vio/pipeline/MonoImuPipeline.h>
#include <kimera-vio/pipeline/StereoImuPipeline.h>
#include <kimera-vio/utils/Timer.h>

// Dependencies from this repository
#include "kimera_vio_ros/RosBagDataProvider.h"
#include "kimera_vio_ros/RosDataProviderInterface.h"
#include "kimera_vio_ros/RosOnlineDataProvider.h"

namespace VIO {

KimeraVioRos::KimeraVioRos()
    : nh_private_("~"),
      vio_params_(nullptr),
      vio_pipeline_(nullptr),
      ros_display_(nullptr),
      ros_visualizer_(nullptr),
      data_provider_(nullptr),
      restart_vio_pipeline_srv_(),
      restart_vio_pipeline_(false) {
  // Add rosservice to restart VIO pipeline if requested.
  restart_vio_pipeline_srv_ = nh_private_.advertiseService(
      "restart_kimera_vio", &KimeraVioRos::restartKimeraVio, this);

  // Parse VIO parameters
  std::string params_folder_path;
  CHECK(nh_private_.getParam("params_folder_path", params_folder_path));
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
  ros_visualizer_.reset();

  VLOG(1) << "Creating Ros Display.";
  CHECK(vio_params_);
  ros_display_ = VIO::make_unique<RosDisplay>();
  ros_visualizer_ = VIO::make_unique<RosVisualizer>(*vio_params_);

  VLOG(1) << "Destroy Vio Pipeline.";
  vio_pipeline_.reset();

  // Second, destroy dataset parser.
  VLOG(1) << "Destroy Data Provider.";
  data_provider_.reset();

  // Then, create dataset parser. This must be before vio pipeline bcs
  // the data provider may modify the init gt pose.
  VLOG(1) << "Creating Data Provider.";
  data_provider_ = createDataProvider(*vio_params_);
  CHECK(data_provider_) << "Data provider construction failed.";

  // Then, create Kimera-VIO from scratch.
  VLOG(1) << "Creating Kimera-VIO.";
  CHECK(ros_display_);

  vio_pipeline_ = nullptr;
  switch (vio_params_->frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline_ = VIO::make_unique<VIO::MonoImuPipeline>(
          *vio_params_, std::move(ros_visualizer_), std::move(ros_display_));
    } break;
    case VIO::FrontendType::kStereoImu: {
      vio_pipeline_ = VIO::make_unique<VIO::StereoImuPipeline>(
          *vio_params_, std::move(ros_visualizer_), std::move(ros_display_));
    } break;
    default: {
      LOG(FATAL) << "Unrecognized frontend type: "
                 << VIO::to_underlying(vio_params_->frontend_type_)
                 << ". 0: Mono, 1: Stereo.";
    } break;
  }
  
  CHECK(vio_pipeline_) << "Vio pipeline construction failed.";

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
    // TODO(Toni): Technically, we can spare a thread with online dataprovider
    // since we can simply call .start() on the async spinners at the ctor level
    std::future<bool> data_provider_handle =
        std::async(std::launch::async,
                   &VIO::RosDataProviderInterface::spin,
                   CHECK_NOTNULL(data_provider_.get()));
    std::future<bool> vio_viz_handle =
        std::async(std::launch::async,
                   &VIO::Pipeline::spinViz,
                   CHECK_NOTNULL(vio_pipeline_.get()));
    std::future<bool> vio_pipeline_handle =
        std::async(std::launch::async,
                   &VIO::Pipeline::spin,
                   CHECK_NOTNULL(vio_pipeline_.get()));
    // Run while ROS is ok and vio pipeline is not shutdown.
    ros::Rate rate(20);  // 20 Hz
    while (ros::ok() && !restart_vio_pipeline_) {
      // Print stats at 1hz
      LOG_EVERY_N(INFO, 20) << vio_pipeline_->printStatistics();
      // Mind that if ROS is using sim_time, this will block if /clock
      // is not published (i.e. when pausing the rosbag).
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
    if (!vio_pipeline_->isShutdown()) vio_pipeline_->shutdown();
    LOG(INFO) << "Joining Kimera-VIO thread.";
    vio_pipeline_handle.get();
    LOG(INFO) << "Kimera-VIO thread joined successfully.";
    LOG(INFO) << "Joining Ros Data Provider thread.";
    data_provider_handle.get();
    LOG(INFO) << "Ros Data Provider thread joined successfully.";
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
      // TODO(Toni): right now this will loop forwever unless ROS dies or Ctrl+C
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
  bool online_run = false;
  CHECK(nh_private_.getParam("online_run", online_run));
  if (online_run) {
    // Running ros online.
    return VIO::make_unique<RosOnlineDataProvider>(vio_params);
  } else {
    // Parse rosbag.
    auto rosbag_data_provider =
        VIO::make_unique<RosbagDataProvider>(vio_params);
    rosbag_data_provider->initialize();
    return rosbag_data_provider;
  }
}

void KimeraVioRos::connectVIO() {
  // Register VIO pipeline callbacks
  // Register callback to shutdown data provider in case VIO pipeline
  // shutsdown.
  CHECK(data_provider_);
  CHECK(vio_pipeline_);
  vio_pipeline_->registerShutdownCallback(
      std::bind(&VIO::DataProviderInterface::shutdown,
                std::ref(*CHECK_NOTNULL(data_provider_.get()))));

  // Register Data Provider callbacks
  data_provider_->registerImuSingleCallback(
      std::bind(&VIO::Pipeline::fillSingleImuQueue,
                std::ref(*CHECK_NOTNULL(vio_pipeline_.get())),
                std::placeholders::_1));

  data_provider_->registerImuMultiCallback(
      std::bind(&VIO::Pipeline::fillMultiImuQueue,
                std::ref(*CHECK_NOTNULL(vio_pipeline_.get())),
                std::placeholders::_1));

  data_provider_->registerLeftFrameCallback(
      std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                std::ref(*CHECK_NOTNULL(vio_pipeline_.get())),
                std::placeholders::_1));

  if (vio_params_->frontend_type_ == VIO::FrontendType::kStereoImu) {
    VIO::StereoImuPipeline::UniquePtr stereo_pipeline =
        VIO::safeCast<VIO::Pipeline, VIO::StereoImuPipeline>(
            std::move(vio_pipeline_));

    data_provider_->registerRightFrameCallback(
        std::bind(&VIO::StereoImuPipeline::fillRightFrameQueue,
                  std::ref(*CHECK_NOTNULL(stereo_pipeline.get())),
                  std::placeholders::_1));

    vio_pipeline_ = VIO::safeCast<VIO::StereoImuPipeline, VIO::Pipeline>(
        std::move(stereo_pipeline));
  }
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
