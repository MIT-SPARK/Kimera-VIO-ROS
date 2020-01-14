cmake_minimum_required(VERSION 3.5)
project(kimera_vio_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

# catkin_simple()

# cs_add_executable(${PROJECT_NAME}
#   src/KimeraVioRos.cpp
#   src/RosDataProvider.cpp
#   src/RosBagDataProvider.cpp
#   src/RosOnlineDataProvider.cpp)
# target_link_libraries(${PROJECT_NAME} kimera_vio)

ament_package()
