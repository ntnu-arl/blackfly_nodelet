#ifndef IMG_EVENT_HANDLER_
#define IMG_EVENT_HANDLER_

// C++

// Spinnaker
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// Nodelet
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// Driver Specific
#include "device_event_handler.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class ImageEventHandler : public ImageEvent
{
 public:
  ImageEventHandler(std::string p_cam_name, CameraPtr p_cam_ptr,
                    image_transport::CameraPublisher *p_cam_pub_ptr,
                    boost::shared_ptr<camera_info_manager::CameraInfoManager>
                        p_c_info_mgr_ptr,
                    DeviceEventHandler *p_device_event_handler_ptr,
                    bool p_exp_time_comp_flag)
  {
    m_cam_name = p_cam_name;
    m_cam_ptr = p_cam_ptr;
    m_cam_pub_ptr = p_cam_pub_ptr;
    m_c_info_mgr_ptr = p_c_info_mgr_ptr;
    m_device_event_handler_ptr = p_device_event_handler_ptr;
    m_last_image_stamp = ros::Time(0, 0);
    m_exp_time_comp_flag = p_exp_time_comp_flag;
    image_msg = boost::make_shared<sensor_msgs::Image>();
    // config_all_chunk_data();
  }
  ~ImageEventHandler() { m_cam_ptr = nullptr; }
  void OnImageEvent(ImagePtr image)
  {
    ros::Time image_arrival_time = ros::Time::now();
    // get the last end of exposure envent from the device event handler
    // (exposure time compensated)
    ros::Time last_event_stamp =
        m_device_event_handler_ptr->get_last_exposure_end();
    ros::Time image_stamp;
    // if the last event stamp is 0, no end of exposure event was received,
    // assign the image arrival time instead
    if (last_event_stamp.toSec() == 0.0)
    {
      image_stamp = image_arrival_time;
      ROS_WARN("BLACKFLY NODELET: NO EVENT STAMP ON CAMERA: %s",
               m_cam_name.c_str());
    }
    else
    {
      image_stamp = last_event_stamp;
    }
    // ROS_INFO("Time Diff b/w arrival time and stamp = %f mSec",
    // (image_arrival_time.toSec() - image_stamp.toSec()) * 1000.0);
    if (image->IsIncomplete())
    {
      ROS_ERROR("Blackfly nodelet : Image retrieval failed : image incomplete");
      return;
    }
    if (m_exp_time_comp_flag)
    {
      // get the exposure time
      double exp_time = double(m_cam_ptr->ExposureTime.GetValue());
      // convert to seconds
      exp_time /= 1000000.0;
      // get half the exposure time
      exp_time /= 2.0;
      // subtract from the end of exposure time to get the middle of the
      // exposure
      image_stamp -= ros::Duration(exp_time);
    }
    if (m_cam_pub_ptr->getNumSubscribers() > 0)
    {
      int height = image->GetHeight();
      int width = image->GetWidth();
      int stride = image->GetStride();
      int bits_per_px = image->GetBitsPerPixel();
      PixelFormatEnums pix_format = image->GetPixelFormat();
      if (pix_format == PixelFormat_BGR8)
      {
        sensor_msgs::fillImage(*image_msg, sensor_msgs::image_encodings::BGR8,
                               height, width, stride, image->GetData());
      }
      else if (pix_format == PixelFormat_Mono8)
      {
        sensor_msgs::fillImage(*image_msg, sensor_msgs::image_encodings::MONO8,
                               height, width, stride, image->GetData());
      }
      else
      {
        ROS_ERROR("Unknown pixel format");
        return;
      }
      image_msg->header.frame_id = m_cam_name;
      image_msg->header.stamp = image_stamp;

      // setup the camera info object
      sensor_msgs::CameraInfo::Ptr cam_info_msg =
          boost::make_shared<sensor_msgs::CameraInfo>(
              sensor_msgs::CameraInfo(m_c_info_mgr_ptr->getCameraInfo()));
      cam_info_msg->header.frame_id = m_cam_name;
      cam_info_msg->header.stamp = image_msg->header.stamp;

      // publish the image
      m_cam_pub_ptr->publish(*image_msg, *cam_info_msg,
                             image_msg->header.stamp);
    }
    image->Release();
  }
  void config_all_chunk_data()
  {
    INodeMap &node_map = m_cam_ptr->GetNodeMap();
    CBooleanPtr ptrChunkModeActive = node_map.GetNode("ChunkModeActive");
    if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive))
    {
      ROS_ERROR(
          "Blackfly Nodelet: Unable to activate chunk mode. Timestamps not "
          "compensating for exp time");
      return;
    }
    ptrChunkModeActive->SetValue(true);
    // Retrieve the selector node
    CEnumerationPtr ptrChunkSelector = node_map.GetNode("ChunkSelector");

    if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
    {
      ROS_ERROR(
          "Blackfly Nodelet: Unable to activate chunk mode. Timestamps not "
          "compensating for exp time");
      return;
    }
    // Retrieve entries
    NodeList_t entries;
    ptrChunkSelector->GetEntries(entries);
    for (size_t i = 0; i < entries.size(); i++)
    {
      // Select entry to be enabled
      CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);

      // Go to next node if problem occurs
      if (!IsAvailable(ptrChunkSelectorEntry) ||
          !IsReadable(ptrChunkSelectorEntry))
      {
        continue;
      }

      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());
      std::string chunk_entry_name =
          ptrChunkSelectorEntry->GetSymbolic().c_str();
      // ROS_INFO("Blackfly Nodelet: Enabling %s", chunk_entry_name.c_str());

      // Retrieve corresponding boolean
      CBooleanPtr ptrChunkEnable = node_map.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!IsAvailable(ptrChunkEnable))
      {
        ROS_WARN("Blackfly Nodelet: Chunk Data: %s not available",
                 chunk_entry_name.c_str());
      }
      else if (ptrChunkEnable->GetValue())
      {
        // ROS_INFO("Blackfly Nodelet: Chunk Data: %s enabled",
        // chunk_entry_name.c_str());
      }
      else if (IsWritable(ptrChunkEnable))
      {
        ptrChunkEnable->SetValue(true);
        // ROS_INFO("Blackfly Nodelet: Chunk Data: %s enabled",
        // chunk_entry_name.c_str());
      }
      else
      {
        ROS_WARN("Blackfly Nodelet: Chunk Data: %s not writable",
                 chunk_entry_name.c_str());
      }
    }
    ROS_INFO("Blackfly Nodelet: Successfully Configured Chunk Data");
  }
  CameraPtr m_cam_ptr;

 private:
  sensor_msgs::ImagePtr image_msg;
  DeviceEventHandler *m_device_event_handler_ptr;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> m_c_info_mgr_ptr;
  image_transport::CameraPublisher *m_cam_pub_ptr;
  std::string m_cam_name;
  ros::Time m_last_image_stamp;
  bool m_exp_time_comp_flag = false;
};
#endif  // IMG_EVENT_HANDLER_