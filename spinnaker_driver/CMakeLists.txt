cmake_minimum_required(VERSION 3.2)
project(spinnaker_driver)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  nodelet
  dynamic_reconfigure
)

# Spinnaker
include(cmake/FindSpinnaker.cmake)
message(STATUS "Spinnaker library: ${Spinnaker_LIBRARIES}")
message(STATUS "Spinnaker include: ${Spinnaker_INCLUDE_DIRS}")

# OpenCV
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/Spinnaker.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spinnaker_driver
  CATKIN_DEPENDS roscpp camera_info_manager sensor_msgs std_msgs cv_bridge image_transport nodelet dynamic_reconfigure
  DEPENDS OpenCV
)

include_directories(
  include
  ${Spinnaker_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(spinnaker_driver
src/spinnaker_driver.cpp
)

target_link_libraries(spinnaker_driver
  ${Spinnaker_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(spinnaker_driver_nodelet
  src/spinnaker_driver_nodelet.cpp
)

target_link_libraries(spinnaker_driver_nodelet
  spinnaker_driver
)

add_executable(spinnaker_driver_node
  src/spinnaker_driver_node.cpp
)

target_link_libraries(spinnaker_driver_node
  spinnaker_driver
)
