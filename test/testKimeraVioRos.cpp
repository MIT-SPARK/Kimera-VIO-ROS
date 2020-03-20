#include <glog/logging.h>
#include <gtest/gtest.h>

namespace VIO {

TEST(KimeraVioRosTest, KimeraVioRosTest) {
  // Works?
  EXPECT_TRUE(false);
}

}  // namespace VIO

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  return RUN_ALL_TESTS();
}
