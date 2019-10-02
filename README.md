# Kimera-VIO-ROS

ROS Wrapper for [Kimera](https://github.com/MIT-SPARK/Kimera).

<div align="center">
    <img src="docs/media/Kimera-VIO-ROS_mesh.gif">
</div>

# 1. Installation

## A. Prerequisities

### i. ROS

Install ROS by following [our reference](./docs/ros_installation.md), or the official [ROS website](https://www.ros.org/install/).

### ii. KimeraVIO's dependencies

Follow installation instructions in [KimeraVIO](https://github.com/MIT-SPARK/Kimera-VIO).
Make sure you install **KimeraVIO's dependencies**: GTSAM, OpenCV, OpenGV.

KimeraVIO itself can be installed by cloning **[KimeraVIO](https://github.com/MIT-SPARK/Kimera-VIO)** in your catkin workspace, so you can spare installing KimeraVIO from source (its dependencies must be installed anyway).

## B. KimeraVIO ROS wrapper Installation

### Dependencies
ROS package dependencies are automatically downloaded using rosinstall:
- (catkin simple)[https://github.com/catkin/catkin_simple]
- (pose_graph_tools)[https://github.mit.edu/SPARK/pose_graph_tools] [Required: for visualization of Loop Closures]
- (mesh_rviz_plugins)[https://github.com/ToniRV/mesh_rviz_plugins] [Optional: for visualization of textured 3D Mesh]

If you have the above prerequisites and [SparkVIO](https://github.mit.edu/SPARK/VIO) installed and built, installation of the SparkVIO ROS wrapper should be:

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone git@github.com:MIT-SPARK/Kimera-VIO-ROS.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge kimera_ros/install/kimera_ros.rosinstall
wstool update
```

**Option 1:** Clone [KimeraVIO](https://github.com/MIT-SPARK/Kimera-VIO) in the catkin workspace, the `package.xml` inside KimeraVIO will let catkin know that it must be installed using cmake:
```bash
git clone git@github.com:MIT-SPARK/Kimera-VIO.git
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
  roslaunch kimera_ros kimera_ros_euroc.launch
  ```

  3. In another terminal, launch rviz for visualization:
  ```bash
  rviz -d $(rospack find kimera_ros)/rviz/kimera_vio_euroc.rviz
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
  roslaunch kimera_ros kimera_ros_euroc.launch online:=false rosbag_path:="PATH/TO/ROSBAG"
  ```

## Other datasets
The launch file and parameters can also be configured for other datasets. For example, here we provide a [kitti rosbag for testing](https://drive.google.com/drive/folders/1mPdc1XFa5y1NrZtffYTkrkGaxj5wvX0T?usp=sharing). To run, in one terminal, launch the spark vio ROS wrapper with the launch file we configured for kitti:
```
roslaunch kimera_ros kimera_ros_kitti.launch
```
  - In another terminal, launch a Kitti rosbag:
```
rosbag play --clock /PATH/TO/KITTI_ROSBAG
```
  - In rviz, you can use the provided config file provided at rviz/kimera_vio_kitti.rviz
  ```bash
  rviz -d $(rospack find kimera_ros)/rviz/spark_vio_kitti.rviz
  ```

# Hardware use

See the [documentation on hardware setup](docs/hardware_setup.md) for instructions on running KimeraROS on supported hardware platforms, as well as guides on how to develop for other platforms.

# BSD License
KimeraVIO ROS wrapper is open source under the BSD license, see the [LICENSE.BSD](./LICENSE.BSD) file.
