/**
 * @file   testKimeraVioRos.cpp
 * @brief  Unit test stub and entry point.
 * @author Antoni Rosinol
 * @author Andrew Violette
 */

#include <glog/logging.h>
#include <gtest/gtest.h>
#include "kimera_vio_ros/KimeraVioRos.h"
// Executable is at ./devel/lib/kimera_vio_ros/testKimeraVioRos
// Executable is run from ./build/kimera_vio_ros
DEFINE_string(test_data_path, "../../src/Kimera-VIO-ROS/test/data",
           "Path to data for unit tests.");

namespace VIO {

}  // namespace VIO

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  // Initialize ROS node.
  ros::init(argc, argv, "kimera_vio");

  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  return RUN_ALL_TESTS();
}
