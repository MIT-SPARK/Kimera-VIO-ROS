cmake_minimum_required(VERSION 2.8.3)
project(kimera_ros)

find_package(catkin_simple REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

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
