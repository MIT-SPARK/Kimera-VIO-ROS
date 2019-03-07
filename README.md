# SPARK_VIO_ROS
ROS Wrapper for [SPARK VIO](https://github.mit.edu/SPARK/VIO).

# Installation
Note that this is o be used with the `feature/kitti_dataset` branch of SparkVio. If you have SparkVio installed and made, installation should just be: (in your catkin_ws/src) 
```
git clone git@github.mit.edu:SPARK/spark_vio_ros.git
cd ..
catkin build
```

# Usage
An example one can try is with the EuRoC dataset. To run, type 
`roslaunch spark_vio_ros spark_vio_ros_euroc.launch data:="<path-to-rosbag>"`

To use your own dataset, you can copy the EuRoC folder and exchange all the values within the folder to those corresponding to your dataset (calibration, topic name, tracker/vio values, etc. ). Then, copy the launch file and just exchange the argument for dataset name to the name of your new folder. 

# ToDo
Check Issues and Projects tabs.

# Notes/FAQ
