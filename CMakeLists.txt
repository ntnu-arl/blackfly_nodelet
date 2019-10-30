cmake_minimum_required(VERSION 2.8.3)
project(blackfly)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  nodelet
) 

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS roscpp nodelet camera_info_manager sensor_msgs std_msgs cv_bridge image_transport 
  DEPENDS OpenCV
)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
find_package(Spinnaker QUIET)
if(NOT Spinnaker_FOUND)
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
endif()

message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")

include_directories(SYSTEM
                  ${Spinnaker_INCLUDE_DIRS}
                  ${catkin_INCLUDE_DIRS}
                  ${OpenCV_INCLUDE_DIRS})
include_directories(include)

#add_library(image_event_handler src/image_event_handler.cpp)
#add_library(device_event_handler src/device_event_handler.cpp)

## Create the nodelet blackfly library
add_library(${PROJECT_NAME}_nodelet 
  src/blackfly_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
                      #image_event_handler
                      #device_event_handler
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
install(FILES nodelet_plugins.xml blackfly_nodelet.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})