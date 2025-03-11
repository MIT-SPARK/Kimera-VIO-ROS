FROM ros:noetic-ros-base

# Some ROS dependencies require user input for keyboard version.
# This prevents those processes from stalling the install process.
ENV DEBIAN_FRONTEND=noninteractive

ARG URL=https://github.com/MIT-SPARK/Kimera-VIO-ROS.git
ARG BRANCH=master
ARG THREADS=4

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y ros-noetic-image-geometry ros-noetic-pcl-ros \
    ros-noetic-cv-bridge git cmake build-essential unzip pkg-config autoconf \
    libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    # Use libvtk5-dev, libgtk2.0-dev in ubuntu 16.04 \
    libvtk7-dev libgtk-3-dev \
    libatlas-base-dev gfortran \
    libparmetis-dev \
    python3-wstool python3-catkin-tools \
    # libtbb recommended for speed: \
    libtbb-dev \
    # for use with rviz
    xvfb

RUN mkdir -p /catkin_ws/src/
RUN git clone ${URL} /catkin_ws/src/Kimera-VIO-ROS
RUN cd /catkin_ws/src/Kimera-VIO-ROS && git checkout origin/${BRANCH}

RUN cd /catkin_ws/src/ && wstool init && \
    wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall && wstool update

# Build catkin workspace
RUN apt-get install -y ros-noetic-image-pipeline ros-noetic-geometry ros-noetic-rviz

RUN . /opt/ros/noetic/setup.sh && cd /catkin_ws && \
        catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
        catkin build -j${THREADS}

# Add to bashrc so user doesn't have to manually do this in every new ROS terminal.
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Resolves an issue where Rviz seg faults on older intel processors
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc