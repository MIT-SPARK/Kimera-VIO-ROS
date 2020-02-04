#include "kimera_vio_ros/KimeraVioNode.hpp"

int main(int argc, char * argv[])
{
  auto g_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  int g_argc = g_args.size();
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&g_argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

//  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
//    rclcpp::executors::SingleThreadedExecutor executor;
  auto kimera_vio_node = std::make_shared<KimeraVioNode>("kimera_node");
  executor.add_node(kimera_vio_node);
  executor.spin();

//  rclcpp::spin(std::make_shared<KimeraVioNode>("kimera_node"));
  rclcpp::shutdown();
  return 0;
}
