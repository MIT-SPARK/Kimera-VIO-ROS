# Kimera-VIO-ROS

ROS Wrapper for [Kimera](https://github.com/MIT-SPARK/Kimera).

<div align="center">
    <img src="docs/media/Kimera-VIO-ROS_mesh.gif">
</div>

## Publications

We kindly ask to cite our paper if you find this library useful:

- A. Rosinol, M. Abate, Y. Chang, L. Carlone, [**Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**](https://arxiv.org/abs/1910.02490). IEEE Intl. Conf. on Robotics and Automation (ICRA), 2020. [arXiv:1910.02490](https://arxiv.org/abs/1910.02490).
 
 ```bibtex
 @InProceedings{Rosinol20icra-Kimera,
   title = {Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping},
   author = {Rosinol, Antoni and Abate, Marcus and Chang, Yun and Carlone, Luca},
   year = {2020},
   booktitle = {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
   url = {https://github.com/MIT-SPARK/Kimera},
   pdf = {https://arxiv.org/pdf/1910.02490.pdf}
 }
```

# 1. Installation

## A. Prerequisities

- Install ROS by following [our reference](./docs/ros_installation.md), or the official [ROS website](https://www.ros.org/install/).

- ROS non-default dependencies for [mesh_rviz_plugins](https://github.com/ToniRV/mesh_rviz_plugins) (change `melodic` for your ROS distribution):
```bash
sudo apt-get install ros-melodic-image-geometry ros-melodic-pcl-ros ros-melodic-cv-bridge
```

- System dependencies:
First, update package list: `sudo apt-get update`
```bash
sudo apt-get install -y --no-install-recommends apt-utils
sudo apt-get install -y \
      cmake build-essential unzip pkg-config autoconf \
      libboost-all-dev \
      libjpeg-dev libpng-dev libtiff-dev \
# Use libvtk5-dev, libgtk2.0-dev in ubuntu 16.04 \
      libvtk6-dev libgtk-3-dev \
      libatlas-base-dev gfortran \
      libparmetis-dev \
      python-wstool python-catkin-tools \
```

- GTSAM's Optional dependencies (highly recommended for speed)
Install [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/): `sudo apt-get install libtbb-dev`

## B. KimeraVIO ROS wrapper Installation

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
# On Ubuntu 16.04:
# catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON
catkin config --merge-devel

# Add workspace to bashrc for automatic sourcing of workspace.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone git@github.com:MIT-SPARK/Kimera-VIO-ROS.git

# Install dependencies from rosinstall file using wstool
wstool init # Use unless wstool is already initialized

# For ssh:
wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_ssh.rosinstall
# For https
# wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall

# Finally, download and update repos:
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build

# Refresh workspace
source ~/catkin_ws/devel/setup.bash
```

# 2. Usage
Download a [Euroc](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) rosbag: for example [V1_01_easy](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag).

## Online
  1. As a general good practice, open a new terminal and run: `roscore`

  2. In another terminal, launch KimeraVIO ROS wrapper:
  ```bash
  roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch
  ```

  3. In another terminal, launch rviz for visualization:
  ```bash
  rviz -d $(rospack find kimera_vio_ros)/rviz/kimera_vio_euroc.rviz
  ```
  > Note: this rviz configuration makes use of a rviz plugin: [mesh_rviz_plugins](https://github.com/ToniRV/mesh_rviz_plugins). To visualize the textured 3D mesh, clone this plugin to your catkin workspace and catkin build it (note that this should be done automatically via `wstool`).

  4. Finally, in another terminal, launch the downloaded Euroc rosbag:
  ```bash
  rosbag play --clock /PATH/TO/EUROC_ROSBAG
  ```

  > Note that you will need to both source ROS and your `catkin_ws` for each new terminal unless you added the following lines to your `~/.bashrc` file:
  > ```bash
  > source /opt/ros/melodic/setup.bash  # Change `melodic` for your ROS distribution.
  > source ~/catkin_ws/devel/setup.bash # Change `bash` to the shell you use.
  > ```

## Offline
  In this mode, the provided rosbag will be first parsed and then sent to the VIO for processing.
  This is particularly useful when debugging to avoid potential ROS networking issues.
  - To run, launch the KimeraVIO ROS wrapper with the `online` parameter set to `false` and specify the rosbag's path:
  ```bash
  roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch online:=false rosbag_path:="PATH/TO/ROSBAG"
  ```

## Other datasets
The launch file and parameters can also be configured for other datasets. For example, here we provide a [kitti rosbag for testing](https://drive.google.com/drive/folders/1mPdc1XFa5y1NrZtffYTkrkGaxj5wvX0T?usp=sharing). To run, in one terminal, launch the Kimera ROS wrapper with the launch file we configured for kitti:
```
roslaunch kimera_vio_ros kimera_vio_ros_kitti.launch
```
  - In another terminal, launch a Kitti rosbag:
```
rosbag play --clock /PATH/TO/KITTI_ROSBAG
```
  - In rviz, you can use the provided config file provided at rviz/kimera_vio_kitti.rviz
  ```bash
  rviz -d $(rospack find kimera_vio_ros)/rviz/kimera_vio_ros_kitti.rviz
  ```

## Running Unit tests

To run unit tests using catkin for this specific package, call (after building the package and sourcing the workspace):

```bash
catkin run_tests --no-deps --this
```

# Hardware use

See the [documentation on hardware setup](docs/hardware_setup.md) for instructions on running KimeraROS on supported hardware platforms, as well as guides on how to develop for other platforms.

# BSD License
KimeraVIO ROS wrapper is open source under the BSD license, see the [LICENSE.BSD](./LICENSE.BSD) file.
