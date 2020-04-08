#include <glog/logging.h>
#include <gtest/gtest.h>
#include "kimera_vio_ros/KimeraVioRos.h"
// Executable is at ./devel/lib/kimera_vio_ros/testKimeraVioRos
DEFINE_string(test_data_path, "../../../Kimera-VIO-ROS/tests/data",
           "Path to data for unit tests.");

namespace VIO {

TEST(KimeraVioRosTest, KimeraVioRosTest) {
  // Works?
  EXPECT_TRUE(false);
}

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
