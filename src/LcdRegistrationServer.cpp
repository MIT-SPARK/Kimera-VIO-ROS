#include "kimera_vio_ros/LcdRegistrationServer.h"
#include <tf2_eigen/tf2_eigen.h>

namespace VIO {

using pose_graph_tools_msgs::LcdFrameRegistration;

LcdRegistrationServer::LcdRegistrationServer(LcdModule* lcd_module)
    : nh_("~"), lcd_module_(CHECK_NOTNULL(lcd_module)) {
  nh_.setCallbackQueue(&callback_queue_);
  server_ = nh_.advertiseService(
      "register_lcd_frames", &LcdRegistrationServer::srvCallback, this);

  worker_thread_.reset(new std::thread(&LcdRegistrationServer::spin, this));
}

void LcdRegistrationServer::stop() {
  should_shutdown_ = true;
  if (worker_thread_) {
    worker_thread_->join();
  }
  worker_thread_.reset();
}

void LcdRegistrationServer::spin() {
  ros::WallRate r(100);
  while (ros::ok() && !should_shutdown_) {
    callback_queue_.callAvailable(ros::WallDuration(0));
    r.sleep();
  }
}

bool LcdRegistrationServer::srvCallback(LcdFrameRegistration::Request& req,
                                        LcdFrameRegistration::Response& res) {
  VLOG(1) << "[LCD] registering " << req.query << " -> " << req.match;
  auto result = lcd_module_->registerFrames(req.query, req.match);
  res.valid = result.isLoop();
  tf2::convert(result.relative_pose_.translation(), res.match_T_query.position);
  Eigen::Quaterniond match_R_query(result.relative_pose_.rotation().matrix());
  tf2::convert(match_R_query, res.match_T_query.orientation);

  return true;
}

}  // namespace VIO