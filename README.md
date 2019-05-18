# SPARK_VIO_ROS
ROS Wrapper for [SPARK VIO](https://github.mit.edu/SPARK/VIO).

# Installation
Note that this is to be used with the `feature/parallelization/kitti_dataset` branch of SparkVio. If you have SparkVio installed and made, installation should just be: (in your catkin_ws/src) 
```
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

cd src
# Clone repo
git clone git@github.mit.edu:SPARK/spark_vio_ros.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge spark_vio_ros/install/spark_vio.rosinstall
wstool update
cd ../

# Install required libraries.
rosdep install --from-paths src --ignore-src --rosdistro melodic -y

cd ..
catkin build

# Refresh workspace
source ~/.bashrc
```

# Usage
An example one can try is with the EuRoC dataset. To run, type 
```
roslaunch spark_vio_ros spark_vio_ros_euroc.launch data:="<path-to-rosbag>" rate:="<playback rate factor>
```
Note that the data parameter is required and the rate is default set to 1.0 (real time)

To use your own dataset, you can copy the param/EuRoC folder and exchange all the values within the folder to those corresponding to your dataset (calibration, topic name, tracker/vio values, etc. ). Then, copy the launch file and just exchange the argument for dataset name to the name of your new folder. 

For debugging, the VERBOSITY argument in the launch file can be toggled. 

You can also run this offline (the rosbag is parsed before starting the pipeline). To do this, type
```
roslaunch spark_vio_ros spark_vio_ros_euroc_offline.launch data:="<path-to-rosbag>"
```
You can use your own dataset, as explained above. 

# ToDo
Check Issues and Projects tabs.

# Notes/FAQ
One possible source of confusion is the DUMMY_DATASET_PATH argument. This is needed because of the way the SparkVio architecture is currently setup. More precisely, it requires the ETH Parser to be passed into the pipeline, so the quick way around it is to give it a dummy eth dataset (placed in the temp folder), that it doesn't really use. 

Another thing to note is that in regularVioParameters.yaml, autoinitialize needs to be set to 1, otherwise the pipeline will initialize according to the ground truth in the dummy data. 
