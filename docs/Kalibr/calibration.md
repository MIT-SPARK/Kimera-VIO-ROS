### Kalibr Camera IMU Calibration

This setup assumes the IMU is already calibrated (see step 5 [here](../hardware_setup.md)).  Additionally, you will need to know the noise density and random walk of the accelerometer and gyroscope.  We've included a `realsense_imu.yaml` file that has been used for calibrating a RealSense D435i camera. We found these value in [this](https://github.com/IntelRealSense/realsense-ros/issues/563) Github issue, but you may be interested in using [these](https://github.com/rpng/kalibr_allan) scripts to get the IMU instrinsics yourself.  If you are have updated values for your specific IMU please update the file accordingly.

1. Collect calibration bagfiles for camera intrinsics and extrinsics [(see instructions)](https://www.youtube.com/watch?v=puNXsnrYWTY&app=desktop).  Use the Aprilgrid 6x6 0.8x0.8m (AO page) that can be downloaded [here](https://github.com/ethz-asl/kalibr/wiki/downloads) from Kalibr. Also download the corresponding yaml file.

2. Calibrate camera intrinsics and extrinsics using [Kalibr](https://github.com/ethz-asl/kalibr).  You can try using the CDE package, although we had difficulties with this, so recommend using [this](https://hub.docker.com/r/davvdg/ros-kalibr) docker container instead. The commands below assume you are using Kalibr with docker.

Pull the docker image and run it.
```bash
# You may need to use sudo for docker commands depending on your installation
docker pull davvdg/ros-kalibr
docker run -it davvdg/ros-kalibr bash
```
Update Kalibr in the container.
```bash
cd kalibr_workspace/src/Kalibr
git pull
cd ../../
catkin build # This make some time to build

# Make a folder for putting calibration files
mkdir calibration
```

In a new terminal, copy your bag file from step 1 into the container.

```bash
docker ps # Find the container ID
docker cp <path-to-bag> <CONTAINER ID>:/kalibr_workspace/calibration/

# Also copy in realsense_imu.yaml file.
docker cp <path-to-realsense_imu.yaml> <CONTAINER ID>:/kalibr_workspace/calibration/

# Copy in the aprilgrid yaml file downloaded in step 1
docker cp <path-to-aprilgrid-yaml> <CONTAINER ID>:/kalibr_workspace/calibration/
```

Now, go back to the container in the first terminal.

```bash
# Source the ros workspace
source devel/setup.bash
cd calibration

# Start by doing a camera calibration to estiamte the camera instrinsics and extrinsics
kalibr_calibrate_camera --bag <bag-file> --topics /camera/infra1/image_rect_raw
/camera/infra2/image_rect_raw --models pinhole-radtan pinhole-radtan --target
<apriltags-yaml-file>
```

This may take a while to run.  Since we are running in docker, the viewer for calibration plots will not open and likely print some errors.  However, the results will still be saves in .txt files.  A `camchain-<bagname>.yaml` file should have also been created to store the calibration results.

We used the `pinhole-radtan` distortion model which is better for low-distortion cameras according to [this](https://docs.openvins.com/gs-calibration.html) page. `pinhole-omni` may produce better results.  Final reprojection errors should be less than 0.2 - 0.5 pixels in a good calibration.

Now that the camera is calibrated, we must calculate the temporal and spatial transformations between the camera and IMU. 

```bash
# Use the camchain file created in the previous step and the realsense_imu.yaml file provided (and adapted)
kalibr_calibrate_imu_camera --bag <bag-file> --cam camchain-<bagname>.yaml --imu realsense_imu.yaml --target
<apriltags-yaml-file>
```

This should create a `camchain-imucam-<bagname>.yaml`.  Copy this out of the docker container for use with Kimera.
```bash
# In the 2nd terminal
docker cp <CONTAINER ID>:/kalibr_workspace/calibration/<camchain-imucam-<bagname>.yaml>  <folder-on-desktop>
```

To save the work you've done in the docker container (in case you would like to re-run any of the steps), make sure you commit your docker changes.

```bash
# In the second terminal

docker commit <CONTAINER ID> <username>/ros-kalibr:v1

# Make sure it is saved by checking your images

docker images
```