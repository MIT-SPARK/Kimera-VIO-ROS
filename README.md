# SPARK_VIO_ROS
ROS Wrapper for [SPARK VIO](https://github.mit.edu/SPARK/VIO).

# Installation

# Usage
An example one can try is with the EuRoC dataset. To run, type 
`roslaunch spark_vio_ros spark_vio_ros_euroc.launch rosbag:="<path-to-rosbag>"`

To use your own dataset, you can copy the EuRoC folder and exchange all the values within the folder to those corresponding to your dataset (calibration, topic name, tracker/vio values, etc. ). Then, copy the launch file and just exchange the argument for dataset name to the name of your new folder. 

# ToDo
Check Issues and Projects tabs.

# Notes/FAQ
