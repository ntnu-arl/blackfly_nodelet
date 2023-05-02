// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "spinnaker_driver/spinnaker_driver.hpp"

// Nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace spinnaker_driver
{
class SpinnakerDriverNodelet : public nodelet::Nodelet
{
public:
  SpinnakerDriverNodelet() {}
  ~SpinnakerDriverNodelet() {}

private:
  virtual void onInit()
  {
    pnh_ = getMTPrivateNodeHandle();
    static spinnaker_driver::SpinnakerDriver sd(pnh_);
  }

  ros::NodeHandle pnh_;
};

}  // namespace spinnaker_driver

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(spinnaker_driver::SpinnakerDriverNodelet, nodelet::Nodelet);
