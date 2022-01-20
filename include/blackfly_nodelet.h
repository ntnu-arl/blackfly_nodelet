#ifndef BLACKFLYNODELET_
#define BLACKFLYNODELET_

// C++
#include <vector>
#include <string>
#include <time.h>
#include <pthread.h>

// Spinnaker
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Time.h>

// Nodelet
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// Driver Specific
#include "image_event_handler_impl.h"
#include "device_event_handler_impl.h"
#include "camera.h"

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include "blackfly/BlackFlyConfig.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace blackfly
{
class blackfly_nodelet : public nodelet::Nodelet
{
 public:
  /**
   * @brief Construct a new blackfly nodelet object
   * 
   * This calls the default nodelet constructor which in turn calls onInit
   * 
   */
  blackfly_nodelet() : first_callback(true), Nodelet() {}

  /**
   * @brief Destroy the blackfly nodelet object
   * 
   * Also releases all camera and system instances
   * 
   */
  ~blackfly_nodelet();

  /**
   * @brief Setup the nodelet
   * 
   * Reads parameters, creates instances etc
   * 
   */
  virtual void onInit();

  /**
   * @brief Dynamic reconfigure callback
   * 
   * @param config new config
   * @param level unused (required by api)
   */
  void callback_dyn_reconf(blackfly::BlackFlyConfig &config, uint32_t level);

 private:
  void enable_chunk_data(INodeMap &cam_node_map);
  boost::shared_ptr<camera_info_manager::CameraInfoManager> c_info_mgr_ptr;
  int numCameras;
  SystemPtr system;
  CameraList camList;
  std::vector<blackfly_camera *> m_cam_vect;
  bool first_callback;

  // dynamic reconfigure
  dynamic_reconfigure::Server<blackfly::BlackFlyConfig> *dr_srv;
  dynamic_reconfigure::Server<blackfly::BlackFlyConfig>::CallbackType
      dyn_rec_cb;
};
}  // namespace blackfly
#endif  // BLACKFLYNODELET_
