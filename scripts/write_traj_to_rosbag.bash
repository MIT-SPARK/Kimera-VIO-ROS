# You need to run a roscore before running this script. o
# Otw, the roslaunch command will not return...

DATASET_PATH="/home/tonirv/datasets/uHumans2/"
ROSBAGS=(
  "apartment_scene/uHumans2_apartment_s1_00h"
  #"apartment_scene/uHumans2_apartment_s1_01h"
  # "apartment_scene/uHumans2_apartment_s1_02h"
  # "subway_scene/uHumans2_subway_s1_00h"
  # "subway_scene/uHumans2_subway_s1_24h"
  # "subway_scene/uHumans2_subway_s1_36h"
  # "office_scene/uHumans2_office_s1_00h"
  # "office_scene/uHumans2_office_s1_06h"
  # "office_scene/uHumans2_office_s1_12h"
  # "neighborhood_scene/uHumans2_neighborhood_s1_00h"
  # "neighborhood_scene/uHumans2_neighborhood_s1_24h"
  # "neighborhood_scene/uHumans2_neighborhood_s1_36h"
)
CSV_PATH="/home/tonirv/Documents/uHumans2_VIO/"
PARAMS=(#'5pt'
        #'2pt'
        'DVIO'
        )

for ROSBAG in "${ROSBAGS[@]}"
do
  ROSBAG_PATH="${DATASET_PATH}${ROSBAG}.bag"
  echo "RUNNING EVAL for rosbag path: $ROSBAG_PATH"
  for PARAM in "${PARAMS[@]}"
  do
    echo "RUNNING EVAL for params: $PARAM"
    DVIO_TRAJ_CSV="${CSV_PATH}/${ROSBAG}/$PARAM/traj_vio.csv"
    python /home/tonirv/Code/ROS/kimera_ws/src/Kimera-VIO-ROS/scripts/write_tfs_in_rosbag.py "$ROSBAG_PATH" "$DVIO_TRAJ_CSV" "${DATASET_PATH}${ROSBAG}.dvio.bag"
  done
done
