# Hardware Setup

> NOTE: This documentation is under construction.

## RealSense D435i (Infrared)

Why do we use the infrared cameras on the D435i?
The infrared cameras offer the option to run the [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO) stereo version on monochrome global shutter cameras, which are generally better suited for visual tracking.

### Setup and Calibration

1. Download and install the [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)

2. Download and install the [Intel RealSense ROS wrapper](https://github.com/IntelRealSense/realsense-ros)

3. Adapt the RealSense ROS wrapper to publish a single interpolated IMU message in the launch file. See [nodelet xml](https://github.com/IntelRealSense/realsense-ros/blob/c2448916218ccfe49b0d642563493cb4e9bdcc3b/realsense2_camera/launch/includes/nodelet.launch.xml#L82) and the [README](https://github.com/IntelRealSense/realsense-ros/blob/development/README.md#launch-parameters) for more information. You may also want to synchronize all the camera messages to be sent with the same timestamp.

```bash
## Uniting the IMU can be done by setting 
<arg name="unite_imu_method" default="linear_interpolation"/>

## Synching camera messages can be done by setting
<arg name="enable_sync" default="true"/>
```

4. Make sure to properly cover the infrared projector on the RealSense (this otherwise affects the quality of the infrared image with dots)

5. Calibrate the IMU instrinsics (see instructions [here](https://github.com/IntelRealSense/librealsense/tree/master/tools/rs-imu-calibration)).  The D435i IMUs do not come calibrated from the factory, and this step impacts the next calibration steps, as the camera scales IMU messages according to the intrinsic values.   More information on the calibration can be found [here](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/RealSense_Depth_D435i_IMU_Calib.pdf). 

6. Follow the Kalibr calibration step described [here](Kalibr/calibration.md).  For more information, visit the [Kalibr Wiki](https://github.com/ethz-asl/kalibr/wiki).

7. Create configuration files for Kimera-VIO-ROS wrapper using [Kalibr2KimeraVIO-pinhole-radtan](https://github.com/MIT-SPARK/Kimera-VIO/tree/master/kalibr/config2kimeravio.py)


```bash
## Locate the camchain-imucam-<bagname>.yaml and realsense_imu.yaml files from step 6. 

## Use the config2kimeravio.py file in Kimera-VIO/kalibr

python ~/Documents/VNAV/agrobot/src/Kimera-VIO/kalibr/config2kimeravio.py -config 'stereo-radtan' -input_cam camchain-imucam-<bagname>.yaml -input_imu realsense_imu.yaml -output <path-to-configuration-file> -responsible 'John Smith' -date '12.03.2019' -camera 'RealSense D435i' -IMU 'RealSense D435i'
```
Copy the created `calibration.yaml` file to the `Kimera-VIO-ROS/param` folder you are using.

8. Create/adapt your own specific launch file, similar to [example RealSense IR](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/launch/kimera_ros_realsense_IR.launch)


### Testing

1. Launch RealSense camera using ```roslaunch realsense2_camera [name of your launch file]```

2. Visualize image stream using ```rosrun image_view image_view image:=[name of camera topic]```

3. Launch Kimera-VIO ROS wrapper using ```roslaunch spark_vio_ros [name of your launch file]```

4. Visualize trajectory with RVIZ using ```rviz```, [(see example config)](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/rviz/kimera_vio_euroc.rviz)

5. Visualize state and statistics using ```rqt_multiplot```, [(see example config)](https://github.com/MIT-SPARK/Kimera-VIO-ROS/tree/master/cfg/viz/rqt_multiplot_state.xml)

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
