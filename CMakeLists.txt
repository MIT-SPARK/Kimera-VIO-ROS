cmake_minimum_required(VERSION 2.8.3)
project(spark_vio_ros)
add_compile_options(-std=c++14)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(stereo_vio_ros
  src/stereoVIOROS.cpp
  src/RosBaseDataSource.cpp
  src/RosbagDataSource
  src/RosDataSource.cpp
  src/StereoImageBuffer.cpp)
target_link_libraries(stereo_vio_ros SparkVio)

cs_install()

cs_export()
