# SPARK_VIO_ROS
ROS Wrapper for [SPARK VIO](https://github.mit.edu/SPARK/VIO).

# Installation
Note that this is to be used with the `feature/parallelization/jpl` branch of SparkVio. If you have SparkVio installed and made, installation should just be: (in your catkin_ws/src) 
```
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd src
git clone git@github.mit.edu:SPARK/spark_vio_ros.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge spark_vio_ros/install/spark_vio.rosinstall
wstool update

# Compile code
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

# Hardware use
## RealSense D435i (Infrared)

Why do we use the infrared cameras on the D435i? 
The infrared cameras offer the option to run the SparkVIO stereo version on monochrome global shutter cameras, which are generally better suited for visual tracking.

### Setup

1. Download and install the [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)

2. Download and install the [Intel RealSense ROS wrapper](https://github.com/IntelRealSense/realsense-ros)

3. Adapt the RealSense ROS wrapper to publish a single interpolated IMU message [(see nodelet xml)](https://github.com/IntelRealSense/realsense-ros/blob/c2448916218ccfe49b0d642563493cb4e9bdcc3b/realsense2_camera/launch/includes/nodelet.launch.xml#L82)

4. Collect calibration bagfiles for camera intrinsics and extrinsics [(see instructions)](https://www.youtube.com/watch?v=puNXsnrYWTY&app=desktop)

5. Calibrate camera intrinsics and extrinsics using [Kalibr](https://github.com/ethz-asl/kalibr)

6. Create configuration files for SparkVIO ROS wrapper using [Kalibr2SparkVIO-pinhole-radtan](https://github.mit.edu/SPARK/VIO/blob/feature/parallelization/jpl/kalibr/kalibr2sparkvio_stereo_pinhole-radtan.py)

7. Create/adapt your own specific launch file, similar to [example RealSense IR](https://github.mit.edu/SPARK/spark_vio_ros/blob/jpl/launch/spark_vio_ros_realsense_IR.launch)

### Testing

1. Launch RealSense camera using ```roslaunch realsense2_camera [name of your launch file]```

2. Visualize image stream using ```rosrun image_view image_view image:=[name of camera topic]```

3. Launch SparkVIO ROS wrapper using ```roslaunch spark_vio_ros [name of your launch file]```

4. Visualize trajectory with RVIZ using ```rviz```, [(see example config)](https://github.mit.edu/SPARK/spark_vio_ros/blob/jpl/viz/visualize_sparkvio.rviz)

5. Visualize state and statistics using ```rqt_multiplot```, [(see example config)](https://github.mit.edu/SPARK/spark_vio_ros/blob/jpl/viz/rqt_multiplot_state.xml)


## MyntEye S

### Setup

1. Download and install the [MyntEye SDK and ROS wrapper](https://github.com/slightech/MYNT-EYE-S-SDK)

2. Collect calibration bagfiles for camera intrinsics and extrinsics [(see instructions)](https://www.youtube.com/watch?v=puNXsnrYWTY&app=desktop)

3. Calibrate camera intrinsics and extrinsics using [Kalibr](https://github.com/ethz-asl/kalibr), recommended model is: ```pinhole-equi``` [(see OpenCV documentation)](https://docs.opencv.org/3.3.1/db/d58/group__calib3d__fisheye.html) 

4. Create configuration files for SparkVIO ROS wrapper using [Kalibr2SparkVIO-pinhole-equi](https://github.mit.edu/SPARK/VIO/blob/feature/parallelization/jpl/kalibr/kalibr2sparkvio_stereo_pinhole-equi.py) or [[Kalibr2SparkVIO-pinhole-radtan](https://github.mit.edu/SPARK/VIO/blob/feature/parallelization/jpl/kalibr/kalibr2sparkvio_stereo_pinhole-equi.py)]

5. Create/adapt your own specific launch file, similar to [example MyntEye S](https://github.mit.edu/SPARK/spark_vio_ros/blob/jpl/launch/spark_vio_ros_mynteye.launch)

### Testing

1. Launch MyntEye camera using ```roslaunch mynt_eye_ros_wrapper [name of your launch file]```

2. Visualize image stream using ```rosrun image_view image_view image:=[name of camera topic]```

3. Launch SparkVIO ROS wrapper using ```roslaunch spark_vio_ros [name of your launch file]``` (example, see below)

4. Visualize trajectory with RVIZ using ```rviz```, [(see example config)](https://github.mit.edu/SPARK/spark_vio_ros/blob/jpl/viz/visualize_sparkvio.rviz)

5. Visualize state and statistics using ```rqt_multiplot```, [(see example config)](https://github.mit.edu/SPARK/spark_vio_ros/blob/jpl/viz/rqt_multiplot_state.xml)

#### Example

For the MyntEyes used in SubT: (online)
```
roslaunch spark_vio_ros spark_vio_ros_mynteye.launch camera:=JPL distortion:=equidistant
```
Options for camera are ```MIT``` and ```JPL```. Options for distortion are ```equidistant``` and ```radtan```.

Same goes for use offline, using the ```spark_vio_ros_mynteye_offline.launch``` file and an additional ```data``` argument with path to bagfile.