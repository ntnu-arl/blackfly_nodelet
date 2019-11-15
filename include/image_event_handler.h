#ifndef IMG_EVENT_HANDLER_
#define IMG_EVENT_HANDLER_
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <pluginlib/class_list_macros.h>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "device_event_handler.h"

using namespace Spinnaker;

class ImageEventHandler : public ImageEvent
{
	public:
		ImageEventHandler(std::string p_cam_name, CameraPtr p_cam_ptr, image_transport::CameraPublisher *p_cam_pub_ptr, 
							boost::shared_ptr<camera_info_manager::CameraInfoManager> p_c_info_mgr_ptr, 
							DeviceEventHandler* p_device_event_handler_ptr)
		{
			m_cam_name = p_cam_name;
			m_cam_ptr = p_cam_ptr;
			m_cam_pub_ptr = p_cam_pub_ptr;
			m_c_info_mgr_ptr = p_c_info_mgr_ptr;
			m_device_event_handler_ptr = p_device_event_handler_ptr;
			m_last_image_stamp = ros::Time(0,0);
			p_cam_ptr = nullptr;
		}
		~ImageEventHandler()
		{
			m_cam_ptr = nullptr;
		}
		void OnImageEvent(ImagePtr image)
		{
			ros::Time image_arrival_time = ros::Time::now();
			// get the last end of exposure envent from the device event handler (already exposure time compensated)
			ros::Time last_event_stamp = m_device_event_handler_ptr->get_last_exposure_end();
			ros::Time image_stamp;
			// if the last event stamp is 0, no end of exposure event was received, assign the image arrival time instead
			if(last_event_stamp.toSec() == 0.0)
			{
				image_stamp = image_arrival_time;
			}
			else
			{
				image_stamp = last_event_stamp;
			}
			// ROS_INFO("Time Diff b/w arrival time and stamp = %f mSec",  (image_arrival_time.toSec() - image_stamp.toSec()) * 1000.0);
			if (image->IsIncomplete())
			{
				ROS_ERROR("Blackfly nodelet : Image retrieval failed : image incomplete");
				return;
			}
			if(m_cam_pub_ptr->getNumSubscribers() > 0)
			{
				sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image>();
				int height = image->GetHeight();
				int width = image->GetWidth();
				int stride = image->GetStride();
				int bits_per_px = image->GetBitsPerPixel();
				//ROS_INFO("H : %i, W : %i, S : %i, B : %i", height, width, stride, bits_per_px);
				PixelFormatEnums pix_format = image->GetPixelFormat();
				//ROS_INFO("Pixel format : %s", image->GetPixelFormatName().c_str());
				if(pix_format == PixelFormat_BGR8)
				{
					sensor_msgs::fillImage(*image_msg, sensor_msgs::image_encodings::BGR8, 
											height, width, stride,
											image->GetData());
				}
				else if(pix_format == PixelFormat_Mono8)
				{
					sensor_msgs::fillImage(*image_msg, sensor_msgs::image_encodings::MONO8, 
											height, width, stride,
											image->GetData());
				}
				else
				{
					ROS_ERROR("Unknown pixel format");
					return;
				}
				image_msg->header.frame_id = m_cam_name;
				image_msg->header.stamp = image_stamp;
				
				// setup the camera info object
				sensor_msgs::CameraInfo::Ptr cam_info_msg = boost::make_shared<sensor_msgs::CameraInfo>(sensor_msgs::CameraInfo(m_c_info_mgr_ptr->getCameraInfo()));
				cam_info_msg->header.frame_id = m_cam_name;
				cam_info_msg->header.stamp = image_msg->header.stamp;

				// publish the image
				m_cam_pub_ptr->publish(*image_msg, *cam_info_msg, image_msg->header.stamp);
			}
			image->Release();
		}
		CameraPtr m_cam_ptr;
	private:
		DeviceEventHandler* m_device_event_handler_ptr;
		boost::shared_ptr<camera_info_manager::CameraInfoManager> m_c_info_mgr_ptr;
		image_transport::CameraPublisher *m_cam_pub_ptr;
		std::string m_cam_name;
		ros::Time m_last_image_stamp;
};
#endif //IMG_EVENT_HANDLER_