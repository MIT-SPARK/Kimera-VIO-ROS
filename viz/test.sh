#!/bin/bash
# Sandro Berchier - 7th May 2019 - MIT
SESSION="SparkVIO_Testing_MIT"

if [ "$#" -gt 0 ]; then
	SENSOR=$1
else
	SENSOR="MyntEye"
	#SENSOR="RealSense"
fi

#################### SETTINGS
DEBUG_IMAGE=true
DEBUG_RVIZ=true
DEBUG_PLOT=false # Need to fix dependency issue with RQT Multiplot
TEST_REINIT=true

DEVEL_FOLDER_ALGO="~/MIT/catkin_ws/devel/setup.bash"
DEVEL_FOLDER_RS="~/MIT/catkin_ws/devel/setup.bash"
DEVEL_FOLDER_MYNT="~/MIT/MYNT-EYE-S-SDK/wrappers/ros/devel/setup.bash"
DEBUG_FOLDER="~/MIT/catkin_ws/src/spark_vio_ros/viz"


#################### SENSOR and ALGORITHM
if [ $SENSOR == "RealSense" ]; then

	#################### SENSOR
	COMMAND_CAMERA="source $DEVEL_FOLDER_RS; roslaunch realsense2_camera rs_D435i_MIT.launch"
	
	#################### SENSOR TOPICS
	CAM0_TOPIC="/realsense/infra1/image_rect_raw"
	CAM1_TOPIC="/realsense/infra2/image_rect_raw"
	IMU0_TOPIC="/realsense/imu"

	#################### ALGORITHM
	COMMAND_ALGORITHM="source $DEVEL_FOLDER_ALGO; roslaunch spark_vio_ros spark_vio_ros_realsense_IR.launch"
	
else

	#################### SENSOR
	COMMAND_CAMERA="source $DEVEL_FOLDER_MYNT; roslaunch mynt_eye_ros_wrapper mynteye_MIT.launch"

	#################### SENSOR TOPICS
	CAM0_TOPIC="/mynteye/right/image_raw"
	CAM1_TOPIC="/mynteye/left/image_raw"
	IMU0_TOPIC="/mynteye/imu/data_raw"

	#################### ALGORITHM
	#COMMAND_ALGORITHM="source $DEVEL_FOLDER_ALGO; roslaunch spark_vio_ros spark_vio_ros_mynteye_MIT.launch"
	COMMAND_ALGORITHM="source $DEVEL_FOLDER_ALGO; roslaunch spark_vio_ros spark_vio_ros_mynteye_equi.launch"

fi


#################### INPUT TOPICS
REINIT_TOPIC="/sparkvio/reinit"


#################### OUTPUT TOPICS
OUTPUT_TOPIC="/sparkvio/odometry"


############################## PANES #########################################

# Split Panes
tmux -2 new-session -d -s $SESSION
#tmux set pane-border-status top
tmux split-window -h  # split vertically
tmux select-pane -t 0
tmux split-window -v -p 33 # split horizontally
tmux select-pane -t 1
tmux split-window -h
tmux select-pane -t 3
tmux split-window -v
tmux select-pane -t 3
tmux split-window -v -p 66
tmux select-pane -t 4
tmux split-window -h -p 33
tmux select-pane -t 4
tmux split-window -h

tmux select-pane -t 7
tmux split-window -v
tmux select-pane -t 7
tmux split-window -v
tmux select-pane -t 9
tmux split-window -v


############################# ROSCORE ########################################

# Exec Commands
#tmux send-keys -t 0 "roscore" C-m

# Wait for ROS Core to Start
#sleep 1


############################### LAUNCH SENSOR #######################################

# Exec Sensor
tmux send-keys -t 3 "$COMMAND_CAMERA" C-m

# Check Rate of Camera 0
tmux send-keys -t 4 "rostopic hz $CAM0_TOPIC" C-m

# Check Rate of Camera 1
tmux send-keys -t 5 "rostopic hz $CAM1_TOPIC" C-m

# Check Rate of IMU 0
tmux send-keys -t 6 "rostopic hz $IMU0_TOPIC" C-m

sleep 5


############################### LAUNCH ALGORITHM #######################################

# Exec Odometry Algorithm
tmux send-keys -t 0 "$COMMAND_ALGORITHM" C-m

sleep 1


############################# VISUALIZE OUTPUT ###################################

# Check Rate of Odometry Estimate
tmux send-keys -t 1 "rostopic hz $OUTPUT_TOPIC" C-m

# Echo dometry Estimate
tmux send-keys -t 2 "rostopic echo $OUTPUT_TOPIC" C-m

# Exec Image View (Left Image)
if $DEBUG_IMAGE; then
	tmux send-keys -t 7 "rosrun image_view image_view image:=$CAM0_TOPIC" C-m
fi

# Exec RQT Multiplot
if $DEBUG_PLOT; then
	tmux send-keys -t 8 "rosrun rqt_multiplot rqt_multiplot --multiplot-config $DEBUG_FOLDER/rqt_multiplot_simulation.xml & rosrun quat2eul quat2eul.py" C-m
fi

# Exec RVIZ
if $DEBUG_RVIZ; then
	tmux send-keys -t 9 "rosrun rviz rviz -d $DEBUG_FOLDER/visualize_sparkvio.rviz" C-m
fi


############################# TEST REINIT ###################################

# Open Chance to Reinit Pipeline
if $TEST_REINIT; then
	tmux send-keys -t 10 "rostopic pub $REINIT_TOPIC std_msgs/Bool '{data: true}'"
	tmux select-pane -t 10
else
	tmux select-pane -t 0
fi


############################### TERMINATE #######################################
tmux -2 attach-session -t $SESSION
