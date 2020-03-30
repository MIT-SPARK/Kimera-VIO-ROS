#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"

namespace VIO {

constexpr char params_folder_path_[] = "";

TEST(RosDataProviderInterfaceTest, ConstructorTest) {
  VIO::VioParams::Ptr dummy_vio_params_ = 
      std::make_shared<VIO::VioParams>(std::string(params_folder_path_));
  //RosDataProviderInterface testInterface(*dummy_vio_params_);
}

}  // namespace VIO
