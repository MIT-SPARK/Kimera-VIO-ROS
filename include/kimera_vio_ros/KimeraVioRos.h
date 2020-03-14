/* @file   KimeraVioRos.cpp
 * @brief  ROS Wrapper for Spark-VIO
 * @author Antoni Rosinol
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/utils/Macros.h>

#include "kimera_vio_ros/RosDataProviderInterface.h"

namespace VIO {

class KimeraVioRos {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(KimeraVioRos);
  KIMERA_POINTER_TYPEDEFS(KimeraVioRos);
  KimeraVioRos();
  virtual ~KimeraVioRos() = default;

  bool runKimeraVio();

 protected:
  bool spin();

  VIO::RosDataProviderInterface::UniquePtr createDataProvider();

  void connectVioPipelineAndDataProvider();

  bool restartKimeraVio(std_srvs::Trigger::Request& request,
                        std_srvs::Trigger::Response& response);

 protected:
  VIO::RosDataProviderInterface::UniquePtr data_provider_;
  VIO::Pipeline::UniquePtr vio_pipeline_;
  ros::ServiceServer restart_vio_pipeline_srv_;
  std::atomic_bool restart_vio_pipeline_;
};

}  // namespace VIO
