FROM ros:melodic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y ros-melodic-image-geometry ros-melodic-pcl-ros \
    ros-melodic-cv-bridge git cmake build-essential unzip pkg-config autoconf \
    libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    # Use libvtk5-dev, libgtk2.0-dev in ubuntu 16.04 \
    libvtk6-dev libgtk-3-dev \
    libatlas-base-dev gfortran \
    libparmetis-dev \
    python-wstool python-catkin-tools \
    # libtbb recommended for speed: \
    libtbb-dev

RUN mkdir -p /catkin_ws/src/
RUN git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git /catkin_ws/src/Kimera-VIO-ROS
RUN cd /catkin_ws/src/Kimera-VIO-ROS 
    # && git checkout ___
RUN cd /catkin_ws/src/ && wstool init && \
    wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall && wstool update

# Build catkin workspace
RUN apt-get install -y ros-melodic-image-pipeline ros-melodic-geometry ros-melodic-rviz

RUN . /opt/ros/melodic/setup.sh && cd /catkin_ws && \
    catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    catkin build
