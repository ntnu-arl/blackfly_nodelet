// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef BLACKFLYNODELET_
#define BLACKFLYNODELET_

#include <time.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Time.h"
#include "std_msgs/UInt64.h"

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <pthread.h>

#include <pluginlib/class_list_macros.h>

#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// Nodelet stuff
#include <nodelet/nodelet.h>

#include "device_event_handler_impl.h"
#include "image_event_handler_impl.h"

#include <blackfly/BlackFlyConfig.h>
#include <dynamic_reconfigure/server.h>

#include "camera.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace blackfly
{
class blackfly_nodelet : public nodelet::Nodelet
{
public:
  blackfly_nodelet() : first_callback(true), Nodelet() {}
  ~blackfly_nodelet();
  virtual void onInit();
  void callback_dyn_reconf(blackfly::BlackFlyConfig & config, uint32_t level);

private:
  void enable_chunk_data(INodeMap & cam_node_map);
  boost::shared_ptr<camera_info_manager::CameraInfoManager> c_info_mgr_ptr;
  int numCameras;
  SystemPtr system;
  CameraList camList;
  std::vector<blackfly_camera *> m_cam_vect;
  bool first_callback;
  // dynamic reconfigure
  dynamic_reconfigure::Server<blackfly::BlackFlyConfig> * dr_srv;
  dynamic_reconfigure::Server<blackfly::BlackFlyConfig>::CallbackType dyn_rec_cb;

  const std::string node_ready_param_name_{"/ready"};
};
}  // namespace blackfly
#endif  // BLACKFLYNODELET_
