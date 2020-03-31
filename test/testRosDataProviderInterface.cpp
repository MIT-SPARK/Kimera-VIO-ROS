#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"

namespace VIO {

static constexpr char params_folder_path_[] = "";
/* ************************************************************************** */

class TestRosDataProviderInterface : public ::testing::Test {
 public:
  TestRosDataProviderInterface() :
        publishing_node_(),
        parameter_server_("~")
        {
    setFrameIDs();
    initializeVioParams();
  }

  ~TestRosDataProviderInterface() { }

 protected:
  ros::NodeHandle publishing_node_;
  ros::NodeHandle parameter_server_;
  VIO::VioParams::Ptr dummy_vio_params_;

  void setFrameIDs() {
    parameter_server_.setParam("base_link_frame_id", "dummy_base_link_id");
    parameter_server_.setParam("world_frame_id", "dummy_world_id");
    parameter_server_.setParam("map_frame_id", "dummy_map_id");
    parameter_server_.setParam("left_cam_frame_id", "dummy_left_cam_id");
    parameter_server_.setParam("right_cam_frame_id", "dummy_right_cam_id");
  }

  void initializeVioParams() {
    dummy_vio_params_ = 
        std::make_shared<VIO::VioParams>(std::string(params_folder_path_));
    // need to push two dummy camera params-- assumes a left and right camera
    VIO::CameraParams dummy_cam_params;
    dummy_vio_params_->camera_params_.push_back(dummy_cam_params);
    dummy_vio_params_->camera_params_.push_back(dummy_cam_params);
  }
};

/* ************************************************************************* */

TEST_F(TestRosDataProviderInterface, constructorTest) {
  RosDataProviderInterface test_interface(*dummy_vio_params_);
}

}  // namespace VIO
