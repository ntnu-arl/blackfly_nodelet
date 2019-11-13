#ifndef BLACKFLYNODELET_
#define BLACKFLYNODELET_

#include <vector>
#include <string>
#include <time.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

#include "std_msgs/Float64.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Time.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <pthread.h>

#include <pluginlib/class_list_macros.h>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

// Nodelet stuff
#include <nodelet/nodelet.h>

#include "image_event_handler.h"
#include "device_event_handler.h"

#include "camera.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace blackfly
{
	class blackfly_nodelet : public nodelet::Nodelet
	{
		public:
			blackfly_nodelet() : Nodelet(){}
			~blackfly_nodelet();
			virtual void onInit();
		private:
			void enable_chunk_data(INodeMap &cam_node_map);
			image_transport::ImageTransport* image_transport_ptr;
			boost::shared_ptr<camera_info_manager::CameraInfoManager>c_info_mgr_ptr;
			int numCameras;
			SystemPtr system;
			CameraList camList;
			std::vector<blackfly_camera*> m_cam_vect;
	};
}
#endif // BLACKFLYNODELET_
