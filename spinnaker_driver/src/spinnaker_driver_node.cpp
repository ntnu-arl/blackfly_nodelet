// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "spinnaker_driver/spinnaker_driver.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "spinnaker_driver_node");
  ros::NodeHandle pnh("~");
  spinnaker_driver::SpinnakerDriver sd(pnh);
  ros::spin();
  return 0;
}
