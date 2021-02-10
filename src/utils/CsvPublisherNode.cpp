#include "kimera_vio_ros/utils/CsvPublisher.h"

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "csv_publisher_node");
  VIO::utils::CsvPublisher csv_publisher;
  ros::spin();
}
