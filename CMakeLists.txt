cmake_minimum_required(VERSION 2.8.3)
project(KimeraVIO_ros)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME}
  src/stereo-vio-ros.cpp
  src/base-data-source.cpp
  src/rosbag-data-source.cpp
  src/ros-data-source.cpp
  src/stereo-image-buffer.cpp)
target_link_libraries(${PROJECT_NAME} KimeraVIO)

cs_install()

cs_export()
