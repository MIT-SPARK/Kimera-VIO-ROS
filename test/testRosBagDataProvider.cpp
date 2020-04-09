/*
* Adapted from Vio's testPipeline.cpp'
*
*/

#include <future>
#include <memory>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ros/ros.h>

#include "kimera_vio_ros/RosBagDataProvider.h"
#include "kimera-vio/pipeline/Pipeline.h"

DECLARE_string(test_data_path);

namespace VIO {

class TestRosBagDataProvider : public ::testing::Test {
 public:
  TestRosBagDataProvider()
      : dataset_parser_(nullptr),
        vio_pipeline_(nullptr),
        vio_params_(FLAGS_test_data_path + "/EurocParams") {
    buildOnlinePipeline(vio_params_);
  }
  ~TestRosBagDataProvider() override {
    destroyPipeline();
  }

 protected:
  void SetUp() override {}
  void TearDown() override {}

  void buildOnlinePipeline(const VioParams& vio_params) {
    constexpr int initial_k = 10;
    constexpr int final_k = 80;
    // Needed in order to disconnect previous pipeline in case someone calls
    // this function repeatedly within the same test.
    destroyPipeline();
    LOG(INFO) << "Building pipeline.";
    vio_pipeline_ = VIO::make_unique<Pipeline>(vio_params);
    dataset_parser_ = makeMicroEurocRosbagReader(vio_params);
    connectVioPipeline();
  }

  void buildOfflinePipeline(const VioParams& vio_params) {
    constexpr int initial_k = 10;
    constexpr int final_k = 80;
    // Needed in order to disconnect previous pipeline in case someone calls
    // this function repeatedly within the same test.
    destroyPipeline();
    LOG(INFO) << "Building pipeline.";
    vio_pipeline_ = VIO::make_unique<Pipeline>(vio_params);
    dataset_parser_ = makeMicroEurocRosbagReader(vio_params);
    connectVioPipelineWithBlockingIfFullQueues();
  }

  VIO::RosbagDataProvider::UniquePtr 
  makeMicroEurocRosbagReader(const VioParams& vio_params) {
    ros::NodeHandle parameter_server("~");

    parameter_server.setParam("rosbag_path", FLAGS_test_data_path 
                              + "/MicroEurocRosbag/MicroEuroc.bag");
    // For dataprovider
    parameter_server.setParam("left_cam_rosbag_topic", "/cam0/image_raw");
    parameter_server.setParam("right_cam_rosbag_topic", "/cam1/image_raw");
    parameter_server.setParam("imu_rosbag_topic", "/imu0");
    parameter_server.setParam("ground_truth_odometry_rosbag_topic", "");
    parameter_server.setParam("base_link_frame_id", "base_link");
    parameter_server.setParam("world_frame_id", "world");
    parameter_server.setParam("map_frame_id", "map");
    parameter_server.setParam("left_cam_frame_id", "cam0");
    parameter_server.setParam("right_cam_frame_id", "cam1");

    // For others
    parameter_server.setParam("online", "false");
    parameter_server.setParam("use_lcd", "false");

    VIO::RosbagDataProvider::UniquePtr micro_euroc_provider =
        VIO::make_unique<RosbagDataProvider>(vio_params);
    return micro_euroc_provider;
  }

  void connectVioPipeline() {
    LOG(INFO) << "Connecting pipeline.";
    CHECK(dataset_parser_);
    CHECK(vio_pipeline_);

    // Register callback to shutdown data provider in case VIO pipeline
    // shutsdown.
    vio_pipeline_->registerShutdownCallback(std::bind(
        &VIO::DataProviderInterface::shutdown, dataset_parser_.get()));

    // Register callback to vio pipeline.
    dataset_parser_->registerImuSingleCallback(
        std::bind(&VIO::Pipeline::fillSingleImuQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser_->registerLeftFrameCallback(
        std::bind(&VIO::Pipeline::fillLeftFrameQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerRightFrameCallback(
        std::bind(&VIO::Pipeline::fillRightFrameQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
  }

  void connectVioPipelineWithBlockingIfFullQueues() {
    LOG(INFO) << "Connecting pipeline.";
    CHECK(dataset_parser_);
    CHECK(vio_pipeline_);

    // Register callback to shutdown data provider in case VIO pipeline
    // shutsdown.
    vio_pipeline_->registerShutdownCallback(std::bind(
        &VIO::DataProviderInterface::shutdown, dataset_parser_.get()));

    // Register callback to vio pipeline.
    dataset_parser_->registerImuSingleCallback(
        std::bind(&VIO::Pipeline::fillSingleImuQueue,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    // We use blocking variants to avoid overgrowing the input queues (use
    // the non-blocking versions with real sensor streams)
    dataset_parser_->registerLeftFrameCallback(
        std::bind(&VIO::Pipeline::fillLeftFrameQueueBlockingIfFull,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
    dataset_parser_->registerRightFrameCallback(
        std::bind(&VIO::Pipeline::fillRightFrameQueueBlockingIfFull,
                  vio_pipeline_.get(),
                  std::placeholders::_1));
  }

  void destroyPipeline() {
    LOG(INFO) << "Destroying pipeline.";
    // First destroy the VIO pipeline (since this will call the shutdown of
    // the dataset_parser)
    vio_pipeline_.reset();
    // Then destroy the dataset parser.
    dataset_parser_.reset();
  }

 protected:
  DataProviderInterface::UniquePtr dataset_parser_;
  Pipeline::UniquePtr vio_pipeline_;
  VioParams vio_params_;
};

TEST_F(TestRosBagDataProvider, OnlineParallelStartManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(TestRosBagDataProvider, OnlineParallelSpinManualShutdown) {
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle.get());
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(TestRosBagDataProvider, OfflineParallelStartManualShutdown) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle_pipeline.get());
}

TEST_F(TestRosBagDataProvider, OfflineParallelSpinManualShutdown) {
  buildOfflinePipeline(vio_params_);
  ASSERT_TRUE(vio_params_.parallel_run_);
  ASSERT_TRUE(dataset_parser_);
  ASSERT_TRUE(vio_pipeline_);
  auto handle = std::async(std::launch::async,
                           &VIO::DataProviderInterface::spin,
                           dataset_parser_.get());
  auto handle_pipeline =
      std::async(std::launch::async, &VIO::Pipeline::spin, vio_pipeline_.get());
  vio_pipeline_->shutdown();
  // Expect false, since the pipeline has been shut down.
  EXPECT_FALSE(handle.get());
  EXPECT_FALSE(handle_pipeline.get());
}

}  // namespace VIO
