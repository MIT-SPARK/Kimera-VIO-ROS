cmake_minimum_required(VERSION 2.8.3)
project(spark_vio_ros)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME}
  src/stereo-vio-ros.cpp
  src/ros-base-data-source.cpp
  src/rosbag-data-source.cpp
  src/ros-data-source.cpp
  src/stereo-image-buffer.cpp)
target_link_libraries(${PROJECT_NAME} SparkVio)
# Set the C++ standard based on features... rather than like this.
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

cs_install()

cs_export()
