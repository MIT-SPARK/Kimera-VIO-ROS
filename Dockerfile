ARG FROM_IMAGE=ros:eloquent

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# clone underlay source
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS
COPY ./install/underlay.repos ./
RUN vcs import src < underlay.repos

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./install/overlay.repos ./
RUN vcs import src < overlay.repos

# copy ros source
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
# COPY ./ src/Kimera-VIO-ROS
COPY ./install/ros.repos ./
RUN vcs import src < ros.repos

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp
    # && find ./ -name "COLCON_IGNORE" | \
    #   xargs cp --parents -t /tmp

# multi-stage for building
FROM $FROM_IMAGE AS build

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

ENV ROS_DISTRO melodic
# ENV ROS2_DISTRO eloquent

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
      ccache \
      lcov \
      ros-$ROS_DISTRO-ros-core \
    && rm -rf /var/lib/apt/lists/*

# install opencv dependencies
RUN apt-get update && apt-get install -y \
      gfortran \
      libatlas-base-dev \
      libgtk-3-dev \
      libjpeg-dev \
      libpng-dev \
      libtiff-dev \
      libvtk6-dev \
      unzip \
    && rm -rf /var/lib/apt/lists/*

# copy underlay manifests
ENV UNDERLAY_WS /opt/underlay_ws
COPY --from=cache /tmp/underlay_ws $UNDERLAY_WS
WORKDIR $UNDERLAY_WS

# install underlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy underlay source
COPY --from=cache $UNDERLAY_WS ./

# build underlay source
ARG UNDERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
        $UNDERLAY_MIXINS \
      --event-handlers console_direct+

# copy overlay manifests
ENV OVERLAY_WS /opt/overlay_ws
COPY --from=cache /tmp/overlay_ws $OVERLAY_WS
WORKDIR $OVERLAY_WS

# install overlay dependencies
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
        $UNDERLAY_WS/src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
COPY --from=cache $OVERLAY_WS ./

# build overlay source
ARG OVERLAY_MIXINS="release ccache"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
# copy ros manifests
ENV ROS_WS /opt/ros_ws
COPY --from=cache /tmp/ros_ws $ROS_WS
WORKDIR $ROS_WS

# install overlay dependencies
RUN . $OVERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
        $UNDERLAY_WS/src \
        $OVERLAY_WS/src \
      --ignore-src \
      --skip-keys "\
        Boost" \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
COPY --from=cache $ROS_WS ./

# build overlay source
ARG ROS_MIXINS="release ccache"
RUN . $OVERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
        $ROS_MIXINS \
      --event-handlers console_direct+

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$ROS_WS/install/setup.bash"|' \
      /ros_entrypoint.sh