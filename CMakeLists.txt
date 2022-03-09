cmake_minimum_required(VERSION 3.2)
project(blackfly)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)


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

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/BlackFly.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS roscpp nodelet camera_info_manager sensor_msgs std_msgs cv_bridge image_transport dynamic_reconfigure
  DEPENDS OpenCV
)

include(cmake/FindSpinnaker.cmake)

message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")

include_directories(SYSTEM
                  ${Spinnaker_INCLUDE_DIRS}
                  ${catkin_INCLUDE_DIRS}
                  ${OpenCV_INCLUDE_DIRS})
include_directories(include)

## Create the nodelet blackfly library
add_library(${PROJECT_NAME}_nodelet 
  src/blackfly_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
                      ${Spinnaker_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_nodelet
      ${catkin_EXPORTED_TARGETS}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

## Mark the nodelet library for installations
install(TARGETS ${PROJECT_NAME}_nodelet
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
