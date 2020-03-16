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

  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;
  FLAGS_v = 1;

  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
