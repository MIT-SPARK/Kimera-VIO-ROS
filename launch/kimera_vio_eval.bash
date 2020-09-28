DATASET_PATH="~/datasets/uHumans2/"
ROSBAGS=(
  "subway_scene/uHumans2_subway_s1_00h"
  "subway_scene/uHumans2_subway_s1_24h"
  "subway_scene/uHumans2_subway_s1_36h"
  "apartment_scene/uHumans2_apartment_s1_00h"
  "apartment_scene/uHumans2_apartment_s1_01h"
  "apartment_scene/uHumans2_apartment_s1_02h"
  "office_scene/uHumans2_office_s1_00h"
  "office_scene/uHumans2_office_s1_06h"
  "office_scene/uHumans2_office_s1_12h"
  "neighborhood_scene/uHumans2_neighborhood_s1_00h"
  "neighborhood_scene/uHumans2_neighborhood_s1_24h"
  "neighborhood_scene/uHumans2_neighborhood_s1_36h"
)
LOGS_PATH="~/Code/ROS/kimera_ws/src/Kimera-VIO-ROS/output_logs"
OUTPUT_PATH="~/Documents/uHumans2_VIO/"

for ROSBAG in $ROSBAGS
do
  ROSBAG_PATH="${DATASET_PATH}${ROSBAG}.bag"

  roslaunch kimera_vio_ros kimera_vio_ros_tesse.launch dataset_name:="5pt" &
  sleep 10
  rosbag play $ROSBAG_PATH
  cp -r $LOGS_PATH "$OUTPUT_PATH/5pt/$ROSBAG"

  roslaunch kimera_vio_ros kimera_vio_ros_tesse.launch dataset_name:="2pt"
  sleep 10
  rosbag play $ROSBAG_PATH
  cp -r $LOGS_PATH "$OUTPUT_PATH/2pt/$ROSBAG"

  roslaunch kimera_vio_ros kimera_vio_ros_tesse.launch dataset_name:="DVIO"
  sleep 10
  rosbag play $ROSBAG_PATH
  cp -r $LOGS_PATH "$OUTPUT_PATH/DVIO/$ROSBAG"
done
