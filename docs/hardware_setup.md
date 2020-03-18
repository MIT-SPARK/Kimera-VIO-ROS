
# Hardware Setup

## RealSense D435i (Infrared)

Why do we use the infrared cameras on the D435i?
The infrared cameras offer the option to run the [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO) stereo version on monochrome global shutter cameras, which are generally better suited for visual tracking.

### Setup
Note: Only 1. and 2. are necessary if you want to use the default calibration and launch files.

1. Download and install the [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md). The `master` branch currently has an issue with buggy image timestamps-- this is fixed in [this pull request](https://github.com/IntelRealSense/librealsense/pull/5751). You will need to check out the `development` branch with `git checkout development` in order to access this fix.

2. Download and install the [Intel RealSense ROS wrapper](https://github.com/IntelRealSense/realsense-ros).

3. Collect calibration parameters for the RealSense. This is already done for the D435i in VIO's `params/RealSenseIR`.
	- Collect calibration bagfiles for camera intrinsics and extrinsics [(see instructions)](https://www.youtube.com/watch?v=puNXsnrYWTY&app=desktop). 
	- Calibrate camera intrinsics and extrinsics using [Kalibr](https://github.com/ethz-asl/kalibr) 
	- Convert the intrinsics and extrinsics to configuration files for Kimera-VIO-ROS wrapper using [Kalibr2KimeraVIO-pinhole-radtan](https://github.com/MIT-SPARK/Kimera-VIO/tree/master/kalibr/config2kimeravio.py).

4. Create/adapt your own specific launch file, or use the [example RealSense D435i file](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/launch/kimera_vio_ros_realsense_IR.launch).

### Testing
Each command will require its own terminal.

1. Run roscore with ```roscore```

2. Launch RealSense camera using ```roslaunch realsense2_camera rs_camera.launch unite_imu_method:=linear_interpolation``` where `rs_camera.launch` can be repaced with your launch file.

2. Visualize image stream using ```rosrun image_view image_view image:=/camera/infra1/image_rect_raw``` where `/camera/infra1/image_rect_raw` can be repaced with your launch file.

3. The RealSense has an IR emitter on it to improve its RGBD stream. This creates undesirable dots on the infrared images. To fix this, you can either:
	1. Disable the emitter after the RealSense node is up using ```rosrun dynamic_reconfigure dynparam set /camera/stereo_module emitter_enabled 0```
	2. Physically cover the emitter on the RealSense with a piece of tape.

4. Launch Kimera-VIO ROS wrapper using ```roslaunch kimera_vio_ros kimera_vio_ros_realsense_IR.launch``` where `kimera_vio_ros_realsense_IR.launch` can be repaced with your launch file.

5. Visualize trajectory with RVIZ using ```rviz -d $(rospack find kimera_vio_ros)/rviz/kimera_vio_euroc.rviz```, where [kimera_vio_euroc.rviz](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/rviz/kimera_vio_euroc.rviz) can be repaced with your rviz setup file.

6. Visualize state and statistics using ```rqt_multiplot```, [(see example config)](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/cfg/viz/rqt_multiplot_state.xml)

It is important to remember that when launching the VIO, the camera should be standing still and upward (camera fov forward looking).

## MyntEye S

### Setup

1. Download and install the [MyntEye SDK and ROS wrapper](https://github.com/slightech/MYNT-EYE-S-SDK)

2. Collect calibration bagfiles for camera intrinsics and extrinsics [(see instructions)](https://www.youtube.com/watch?v=puNXsnrYWTY&app=desktop)

3. Calibrate camera intrinsics and extrinsics using [Kalibr](https://github.com/ethz-asl/kalibr), recommended model is: ```pinhole-equi``` [(see OpenCV documentation)](https://docs.opencv.org/3.3.1/db/d58/group__calib3d__fisheye.html)

4. Create configuration files for Kimera-VIO-ROS wrapper using [Kalibr2KimeraVIO-pinhole-equi](https://github.com/MIT-SPARK/Kimera-VIO/tree/master/kalibr/config2kimeravio.py) (or pinholde-radtan)

5. Create/adapt your own specific launch file, similar to [example MyntEye S](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/launch/kimera_vio_ros_mynteye.launch)

### Testing

1. Launch MyntEye camera using ```roslaunch mynt_eye_ros_wrapper [name of your launch file]```

2. Visualize image stream using ```rosrun image_view image_view image:=[name of camera topic]```

3. Launch Kimera-VIO-ROS wrapper using ```roslaunch spark_vio_ros [name of your launch file]``` (example, see below)

4. Visualize trajectory with RVIZ using ```rviz```, [(see example config)](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/rviz/kimera_vio_euroc.rviz)

5. Visualize state and statistics using ```rqt_multiplot```, [(see example config)](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/cfg/viz/rqt_multiplot_state.xml)

#### Example

For the MyntEyes used in SubT: (online)
```
roslaunch spark_vio_ros spark_vio_ros_mynteye.launch camera:=JPL distortion:=equidistant
```
Options for camera are ```MIT``` and ```JPL```. Options for distortion are ```equidistant``` and ```radtan```.

Same goes for use offline, using the ```spark_vio_ros_mynteye_offline.launch``` file and an additional ```data``` argument with path to bagfile.

