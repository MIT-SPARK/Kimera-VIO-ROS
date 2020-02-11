#include <glog/logging.h>
#include <gtest/gtest.h>

namespace VIO {

TEST(BaseDataSourceTest, BaseDataSource) {
  // Works?
}

}  // namespace VIO

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
