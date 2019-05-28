#!/bin/bash
#
# /* ----------------------------------------------------------------------------
# * Copyright 2019, Massachusetts Institute of Technology,
# * Cambridge, MA 02139
# * All Rights Reserved
# * Authors: Luca Carlone, et al. (see THANKS for the full author list)
# * See LICENSE for the license information
# * -------------------------------------------------------------------------- */
#
# /**
# * @file   test.sh
# * @brief  test SparkVIO ROS wrapper
# * @author Sandro Berchier
# */
#
##################################################################################################
##################################################################################################
#
# Usage    : ./test.sh flags
# 	arg1 	--> sensor used, options: RealSense, MyntEye, EuRoC
#	arg2 	--> specific camera used, options: MIT, JPL
#	arg3	--> distortion model to be used, options: radtan, equidistant
#	arg4	--> plot type used to debug, options: state, diagnostics
#
# Example:
# 	./test.sh RealSense MIT radtan diagnostics
#	./test.sh RealSense MIT radtan state
#	./test.sh MyntEye JPL equidistant diagnostics
#
##################################################################################################
##################################################################################################

SESSION="SparkVIO_Testing_MIT"

if [ "$#" -gt 0 ]; then
	SENSOR=$1
	CAMERA=$2
	DISTORTION=$3
	DEBUG_PLOT=$4
else
	SENSOR="MyntEye"
	CAMERA="MIT"
	DISTORTION="radtan"
	DEBUG_PLOT="diagnostics"
fi

#################### SETTINGS
DEBUG_IMAGE=true
DEBUG_RVIZ=true
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

elif [ $SENSOR == "RealSense_rgbd" ]; then

	#################### SENSOR
	COMMAND_CAMERA="source $DEVEL_FOLDER_RS; roslaunch realsense2_camera rs_D435i_MIT.launch"
	
	#################### SENSOR TOPICS
	CAM0_TOPIC="/realsense/color/image_raw"
	CAM1_TOPIC="/realsense/aligned_depth_to_color/image_raw"
	IMU0_TOPIC="/realsense/imu"

	#################### ALGORITHM
	COMMAND_ALGORITHM="source $DEVEL_FOLDER_ALGO; roslaunch spark_vio_ros spark_vio_ros_realsense_rgbd.launch"
	
elif [ $SENSOR == "MyntEye" ]; then

	#################### SENSOR
	COMMAND_CAMERA="source $DEVEL_FOLDER_MYNT; roslaunch mynt_eye_ros_wrapper mynteye_MIT.launch"

	#################### SENSOR TOPICS
	CAM0_TOPIC="/mynteye/right/image_raw"
	CAM1_TOPIC="/mynteye/left/image_raw"
	IMU0_TOPIC="/mynteye/imu/data_raw"

	#################### ALGORITHM
	COMMAND_ALGORITHM="source $DEVEL_FOLDER_ALGO; roslaunch spark_vio_ros spark_vio_ros_mynteye.launch camera:=$CAMERA distortion:=$DISTORTION"

else 

	#################### SENSOR
	COMMAND_CAMERA="rosbag play -r 1 -s 22 ~/Dataset/EuRoC/MH_01_easy.bag"

	#################### SENSOR TOPICS
	CAM0_TOPIC="/cam0/image_raw"
	CAM1_TOPIC="/cam1/image_raw"
	IMU0_TOPIC="/imu0/data_raw"

	#################### ALGORITHM
	COMMAND_ALGORITHM="source $DEVEL_FOLDER_ALGO; roslaunch spark_vio_ros spark_vio_ros_euroc.launch"

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
if [ $DEBUG_PLOT == "diagnostics" ]; then
	tmux send-keys -t 8 "source $DEVEL_FOLDER_ALGO; rosrun rqt_multiplot rqt_multiplot --force-discover --multiplot-config $DEBUG_FOLDER/rqt_multiplot_diagnostics.xml" C-m
else
	tmux send-keys -t 8 "source $DEVEL_FOLDER_ALGO; rosrun rqt_multiplot rqt_multiplot --force-discover --multiplot-config $DEBUG_FOLDER/rqt_multiplot_state.xml" C-m
fi

#tmux send-keys -t 8 "source $DEVEL_FOLDER_ALGO; rosrun rqt_multiplot rqt_multiplot --force-discover --multiplot-config $DEBUG_FOLDER/rqt_multiplot_diagnostics.xml & rosrun quat2eul quat2eul.py" C-m

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
