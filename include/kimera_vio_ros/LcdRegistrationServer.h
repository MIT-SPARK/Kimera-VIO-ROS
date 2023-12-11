#pragma once
#include <kimera-vio/loopclosure/LcdModule.h>

#include <pose_graph_tools_msgs/LcdFrameRegistration.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace VIO {

class LcdRegistrationServer {
 public:
  explicit LcdRegistrationServer(LcdModule* lcd_module);

  void stop();

 private:
  void spin();

  bool srvCallback(pose_graph_tools_msgs::LcdFrameRegistration::Request& req,
                   pose_graph_tools_msgs::LcdFrameRegistration::Response& res);

  ros::NodeHandle nh_;
  LcdModule* lcd_module_;

  std::unique_ptr<std::thread> worker_thread_;
  ros::CallbackQueue callback_queue_;
  ros::ServiceServer server_;

  std::atomic<bool> should_shutdown_{false};
};

}  // namespace VIO