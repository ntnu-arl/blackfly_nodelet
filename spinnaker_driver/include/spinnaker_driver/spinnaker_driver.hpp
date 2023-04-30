// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef SPINNAKER_DRIVER_SPINNAKER_DRIVER_HPP_
#define SPINNAKER_DRIVER_SPINNAKER_DRIVER_HPP_

// C++
#include <pthread.h>
#include <time.h>
#include <string>
#include <vector>

// ROS
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt64.h>

// Spinnaker
#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

// spinnaker_driver
#include "spinnaker_driver/camera.hpp"
#include "spinnaker_driver/device_event_handler_impl.hpp"
#include "spinnaker_driver/image_event_handler_impl.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace spinnaker_driver
{
class SpinnakerDriver
{
public:
  SpinnakerDriver(ros::NodeHandle & pnh);
  ~SpinnakerDriver();
  void teardown();

private:
  SystemPtr system_;
  CameraList camera_list_;
  std::vector<std::unique_ptr<Camera>> cameras_;
};
}  // namespace spinnaker_driver
#endif  // SPINNAKER_DRIVER_SPINNAKER_DRIVER_HPP_
