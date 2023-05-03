// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef SPINNAKER_DRIVER_SPINNAKER_DRIVER_HPP_
#define SPINNAKER_DRIVER_SPINNAKER_DRIVER_HPP_

// C++
#include <vector>

// ROS
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

// Spinnaker
#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

// spinnaker_driver
#include "spinnaker_driver/SpinnakerConfig.h"
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
  void dynamicReconfigureCb(spinnaker_driver::SpinnakerConfig & config, uint32_t level);

private:
  SystemPtr system_;
  CameraList camera_list_;
  std::vector<std::unique_ptr<Camera>> cameras_;
};
}  // namespace spinnaker_driver
#endif  // SPINNAKER_DRIVER_SPINNAKER_DRIVER_HPP_
