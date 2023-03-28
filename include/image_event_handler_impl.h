// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef IMG_EVENT_HANDLER_IMPL_
#define IMG_EVENT_HANDLER_IMPL_
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <pluginlib/class_list_macros.h>

#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "device_event_handler_impl.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class ImageEventHandlerImpl : public ImageEventHandler
{
public:
  ImageEventHandlerImpl(
    std::string p_cam_name, CameraPtr p_cam_ptr, image_transport::CameraPublisher * p_cam_pub_ptr,
    boost::shared_ptr<camera_info_manager::CameraInfoManager> p_c_info_mgr_ptr,
    DeviceEventHandlerImpl * p_device_event_handler_ptr, bool p_exp_time_comp_flag,
    std::shared_ptr<std::deque<std_msgs::Header>> p_time_stamp_deque_ptr)
  {
    m_cam_name = p_cam_name;
    m_cam_ptr = p_cam_ptr;
    m_cam_pub_ptr = p_cam_pub_ptr;
    m_c_info_mgr_ptr = p_c_info_mgr_ptr;
    m_device_event_handler_ptr = p_device_event_handler_ptr;
    m_last_image_stamp = ros::Time(0, 0);
    m_exp_time_comp_flag = p_exp_time_comp_flag;
    m_time_stamp_deque_ptr = p_time_stamp_deque_ptr;
    image_msg = boost::make_shared<sensor_msgs::Image>();
    // config_all_chunk_data();
  }
  ~ImageEventHandlerImpl() { m_cam_ptr = nullptr; }
  void OnImageEvent(ImagePtr image)
  {
    ros::Time image_arrival_time = ros::Time::now();
    ros::Time curr_time = m_time_stamp_deque_ptr->back().stamp;
    m_time_stamp_deque_ptr->pop_back();
    ros::Duration sampling_time = curr_time - prev_time_;
    prev_time_ = curr_time;
    std::cout << sampling_time << std::endl;
    // get the last end of exposure envent from the device event handler (exposure time compensated)
    ros::Time last_event_stamp = m_device_event_handler_ptr->get_last_exposure_end();
    ros::Time image_stamp;
    // if the last event stamp is 0, no end of exposure event was received, assign the image arrival
    // time instead
    if (last_event_stamp.toSec() == 0.0) {
      image_stamp = image_arrival_time;
      ROS_WARN(
        "Blackfly Nodelet: No event stamp on camera %s, assigning image arrival time instead",
        m_cam_name.c_str());
    } else {
      image_stamp = last_event_stamp;
    }
    if (image->IsIncomplete()) {
      ROS_ERROR(
        "Blackfly Nodelet: Image retrieval failed: image incomplete for %s", m_cam_name.c_str());
      return;
    }

    if (!m_time_stamp_deque_ptr->empty()) {
      // ros::Time curr_time = m_time_stamp_deque_ptr->back().stamp;
      image_stamp = m_time_stamp_deque_ptr->front().stamp;
      m_time_stamp_deque_ptr->pop_front();
      // ros::Duration sampling_time = curr_time - prev_time_;
      // prev_time_ = curr_time;
      // std::cout << sampling_time << std::endl;
    } else {
      image_stamp = ros::Time(0);
      ROS_WARN("image without trigger");
    }
    if (m_exp_time_comp_flag) {
      // get the exposure time
      double exp_time = double(m_cam_ptr->ExposureTime.GetValue());
      // convert to seconds
      exp_time /= 1000000.0;
      // get half the exposure time
      exp_time /= 2.0;
      // subtract from the end of exposure time to get the middle of the exposure
      image_stamp -= ros::Duration(exp_time);
    }
    if (m_cam_pub_ptr->getNumSubscribers() > 0) {
      int height = image->GetHeight();
      int width = image->GetWidth();
      int stride = image->GetStride();
      int bits_per_px = image->GetBitsPerPixel();
      PixelFormatEnums pix_format = image->GetPixelFormat();
      if (pix_format == PixelFormat_BGR8) {
        sensor_msgs::fillImage(
          *image_msg, sensor_msgs::image_encodings::BGR8, height, width, stride, image->GetData());
      } else if (pix_format == PixelFormat_Mono8) {
        sensor_msgs::fillImage(
          *image_msg, sensor_msgs::image_encodings::MONO8, height, width, stride, image->GetData());
      } else {
        ROS_ERROR("Unknown pixel format");
        return;
      }
      image_msg->header.frame_id = m_cam_name;
      image_msg->header.stamp = image_stamp;

      // setup the camera info object
      sensor_msgs::CameraInfo::Ptr cam_info_msg = boost::make_shared<sensor_msgs::CameraInfo>(
        sensor_msgs::CameraInfo(m_c_info_mgr_ptr->getCameraInfo()));
      cam_info_msg->header.frame_id = m_cam_name;
      cam_info_msg->header.stamp = image_msg->header.stamp;

      // publish the image
      m_cam_pub_ptr->publish(*image_msg, *cam_info_msg, image_msg->header.stamp);
    }
    image->Release();
  }
  void config_all_chunk_data()
  {
    INodeMap & node_map = m_cam_ptr->GetNodeMap();
    CBooleanPtr ptrChunkModeActive = node_map.GetNode("ChunkModeActive");
    if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive)) {
      ROS_ERROR(
        "Blackfly Nodelet: Unable to activate chunk mode. Timestamps not compensating for exp "
        "time");
      return;
    }
    ptrChunkModeActive->SetValue(true);
    // Retrieve the selector node
    CEnumerationPtr ptrChunkSelector = node_map.GetNode("ChunkSelector");

    if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector)) {
      ROS_ERROR(
        "Blackfly Nodelet: Unable to activate chunk mode. Timestamps not compensating for exp "
        "time");
      return;
    }
    // Retrieve entries
    NodeList_t entries;
    ptrChunkSelector->GetEntries(entries);
    for (size_t i = 0; i < entries.size(); i++) {
      // Select entry to be enabled
      CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);

      // Go to next node if problem occurs
      if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry)) {
        continue;
      }

      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());
      std::string chunk_entry_name = ptrChunkSelectorEntry->GetSymbolic().c_str();
      // ROS_INFO("Blackfly Nodelet: Enabling %s", chunk_entry_name.c_str());

      // Retrieve corresponding boolean
      CBooleanPtr ptrChunkEnable = node_map.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!IsAvailable(ptrChunkEnable)) {
        ROS_WARN("Blackfly Nodelet: Chunk Data: %s not available", chunk_entry_name.c_str());
      } else if (ptrChunkEnable->GetValue()) {
        // ROS_INFO("Blackfly Nodelet: Chunk Data: %s enabled", chunk_entry_name.c_str());
      } else if (IsWritable(ptrChunkEnable)) {
        ptrChunkEnable->SetValue(true);
        // ROS_INFO("Blackfly Nodelet: Chunk Data: %s enabled", chunk_entry_name.c_str());
      } else {
        ROS_WARN("Blackfly Nodelet: Chunk Data: %s not writable", chunk_entry_name.c_str());
      }
    }
    ROS_INFO("Blackfly Nodelet: Successfully Configured Chunk Data");
  }
  CameraPtr m_cam_ptr;

private:
  sensor_msgs::ImagePtr image_msg;
  DeviceEventHandlerImpl * m_device_event_handler_ptr;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> m_c_info_mgr_ptr;
  image_transport::CameraPublisher * m_cam_pub_ptr;
  std::string m_cam_name;
  ros::Time m_last_image_stamp;
  bool m_exp_time_comp_flag = false;

  ros::Time prev_time_;
  std::shared_ptr<std::deque<std_msgs::Header>> m_time_stamp_deque_ptr;
};
#endif  // IMG_EVENT_HANDLER_IMPL_
