#include "kimera_vio_ros/KimeraVioNode.hpp"

int main(int argc, char * argv[])
{
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto kimera_vio_node = std::make_shared<KimeraVioNode>();
  executor.add_node(kimera_vio_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
