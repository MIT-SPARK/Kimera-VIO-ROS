#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"

namespace VIO {

// Path for default VioParams parsing
// Empty because default values will serve our purpose
static constexpr char params_folder_path[] = "";

// Constants for ROS TF
static constexpr char dummy_base_link_frame_id_[] = "dummy_base_link_frame_id";
static constexpr char dummy_world_frame_id_[] = "dummy_world_frame_id";
static constexpr char dummy_map_frame_id_[] = "dummy_map_frame_id";
static constexpr char dummy_left_cam_frame_id_[] = "dummy_left_cam_frame_id";
static constexpr char dummy_right_cam_frame_id_[] = "dummy_right_cam_frame_id";

// Expose protected members for testing
// see: StackOverflow's "How do I unit test a protected method in C++?" KrisTC
class RosDataProviderInterfaceExposed : public RosDataProviderInterface {
  public:
    RosDataProviderInterfaceExposed(const VioParams& vio_params) 
      : RosDataProviderInterface(vio_params) {} 
    using RosDataProviderInterface::world_frame_id_;
};


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
    parameter_server_.setParam("base_link_frame_id", dummy_base_link_frame_id_);
    parameter_server_.setParam("world_frame_id", dummy_world_frame_id_);
    parameter_server_.setParam("map_frame_id", dummy_map_frame_id_);
    parameter_server_.setParam("left_cam_frame_id", dummy_left_cam_frame_id_);
    parameter_server_.setParam("right_cam_frame_id", dummy_right_cam_frame_id_);
  }

  void initializeVioParams() {
    dummy_vio_params_ = 
        std::make_shared<VIO::VioParams>(std::string(params_folder_path));
    // need to push two dummy camera params-- assumes a left and right camera
    VIO::CameraParams dummy_cam_params;
    dummy_vio_params_->camera_params_.push_back(dummy_cam_params);
    dummy_vio_params_->camera_params_.push_back(dummy_cam_params);
  }
};

/* ************************************************************************* */

TEST_F(TestRosDataProviderInterface, constructorTest) {
  RosDataProviderInterfaceExposed test_interface(*dummy_vio_params_);

  EXPECT_EQ(dummy_world_frame_id_, test_interface.world_frame_id_);
}

}  // namespace VIO
