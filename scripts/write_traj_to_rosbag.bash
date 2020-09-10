# You need to run a roscore before running this script.
# Otw, the roslaunch command will not return...
# To kill this script: 1. Ctrl+Z 2. kill %%
KIMERA_VIO_ROS_PATH="$HOME/Code/ROS/kimera_ws/src/Kimera-VIO-ROS/"
DATASET_PATH="$HOME/datasets/uHumans2/"
CSV_PATH="$HOME/Documents/uHumans2_VIO/"

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
PARAMS=(#'5pt'
        #'2pt'
        'DVIO'
        )

for ROSBAG in "${ROSBAGS[@]}"
do
  ROSBAG_PATH="${DATASET_PATH}${ROSBAG}.bag"
  echo "RUNNING tf writer for rosbag path: $ROSBAG_PATH"
  for PARAM in "${PARAMS[@]}"
  do
    echo "RUNNING tf writer for params: $PARAM"

    # TODO(Toni): do this in batch... read multiple csvs, and log them all to the rosbag
    echo "Writing VIO tf."
    DVIO_TRAJ_CSV="${CSV_PATH}/${ROSBAG}/$PARAM/traj_vio.csv"
    python $KIMERA_VIO_ROS_PATH/scripts/write_tfs_in_rosbag.py "$ROSBAG_PATH" "$DVIO_TRAJ_CSV" --base_link_frame_id "base_link_${PARAM}"
    echo "Done writing VIO tf."

    echo "Writing PGO tf."
    PGO_TRAJ_CSV="${CSV_PATH}/${ROSBAG}/$PARAM/traj_pgo.csv"
    python $KIMERA_VIO_ROS_PATH/scripts/write_tfs_in_rosbag.py "$ROSBAG_PATH" "$PGO_TRAJ_CSV" --base_link_frame_id "base_link_${PARAM}_pgo"
    echo "Done writing PGO tf."

  done
  echo "Done writing all tfs in rosbag."

done
echo "Done."
