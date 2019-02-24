cmake_minimum_required(VERSION 2.8.3)
project(spark_vio_ros)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(stereo_vio_ros src/stereoVIOROS.cpp src/RosDataSource.cpp)
target_link_libraries(stereo_vio_ros SparkVio::sparkvio)

cs_install()

cs_export()