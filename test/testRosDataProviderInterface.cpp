#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"

namespace VIO {

constexpr char params_folder_path_[] = "";

TEST(RosDataProviderInterfaceTest, ConstructorTest) {
  ros::NodeHandle publishing_node();
  ros::NodeHandle parameter_server("~");
  parameter_server.setParam("base_link_frame_id", "dummy_base_link_id");
  parameter_server.setParam("world_frame_id", "dummy_world_id");
  parameter_server.setParam("map_frame_id", "dummy_map_id");
  parameter_server.setParam("left_cam_frame_id", "dummy_left_cam_id");
  parameter_server.setParam("right_cam_frame_id", "dummy_right_cam_id");
  
  VIO::VioParams::Ptr dummy_vio_params = 
      std::make_shared<VIO::VioParams>(std::string(params_folder_path_));
  // need to push two dummy camera params-- assumes a left and right camera
  VIO::CameraParams dummy_params;
  dummy_vio_params->camera_params_.push_back(dummy_params);
  dummy_vio_params->camera_params_.push_back(dummy_params);

  RosDataProviderInterface testInterface(*dummy_vio_params);
}

}  // namespace VIO
