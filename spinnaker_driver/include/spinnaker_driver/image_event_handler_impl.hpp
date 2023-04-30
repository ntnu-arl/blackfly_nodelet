// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef SPINNAKER_DRIVER_IMAGE_EVENT_HANDLER_IMPL_HPP_
#define SPINNAKER_DRIVER_IMAGE_EVENT_HANDLER_IMPL_HPP_

// ROS
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

// Spinnaker
#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

// spinnaker_driver
#include "spinnaker_driver/device_event_handler_impl.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class ImageEventHandlerImpl : public ImageEventHandler
{
public:
  ImageEventHandlerImpl(
    std::string & cam_name, CameraPtr cam_ptr, image_transport::CameraPublisher & cam_pub,
    sensor_msgs::CameraInfo & cam_info_msg,
    std::shared_ptr<DeviceEventHandlerImpl> device_event_handler, bool exp_time_comp_flag)
  {
    cam_name_ = cam_name;
    cam_ptr_ = cam_ptr;
    cam_pub_ = cam_pub;
    device_event_handler_ = device_event_handler;
    last_image_stamp_ = ros::Time(0, 0);
    exp_time_comp_flag_ = exp_time_comp_flag;
    image_msg_.header.frame_id = cam_name_;
    cam_info_msg_ = cam_info_msg;
    cam_info_msg_.header.frame_id = cam_name_;
  }

  ~ImageEventHandlerImpl() {}

  void OnImageEvent(ImagePtr image)
  {
    ros::Time image_arrival_time = ros::Time::now();
    // get the last end of exposure envent from the device event handler (exposure time compensated)
    ros::Time last_event_stamp = device_event_handler_->getLastExposureEnd();
    ros::Time image_stamp;
    // if the last event stamp is 0, no end of exposure event was received, assign the image arrival
    // time instead
    if (last_event_stamp.toSec() == 0.0) {
      image_stamp = image_arrival_time;
      ROS_WARN(
        "No event stamp on camera %s, assigning image arrival time instead", cam_name_.c_str());
    } else {
      image_stamp = last_event_stamp;
    }

    if (image->IsIncomplete()) {
      ROS_ERROR("Image retrieval failed: image incomplete for %s", cam_name_.c_str());
      return;
    }

    if (exp_time_comp_flag_) {
      // get the exposure time in microseconds
      double exp_time = double(cam_ptr_->ExposureTime.GetValue());
      // convert to seconds
      exp_time /= 1.0e6;
      // get half the exposure time
      exp_time /= 2.0;
      // subtract from the end of exposure time to get the middle of the exposure
      image_stamp -= ros::Duration(exp_time);
    }

    if (cam_pub_.getNumSubscribers() > 0) {
      int height = image->GetHeight();
      int width = image->GetWidth();
      int stride = image->GetStride();
      int bits_per_px = image->GetBitsPerPixel();
      PixelFormatEnums pix_format = image->GetPixelFormat();
      if (pix_format == PixelFormat_BGR8) {
        sensor_msgs::fillImage(
          image_msg_, sensor_msgs::image_encodings::BGR8, height, width, stride, image->GetData());
      } else if (pix_format == PixelFormat_Mono8) {
        sensor_msgs::fillImage(
          image_msg_, sensor_msgs::image_encodings::MONO8, height, width, stride,
          image->GetData());
      } else {
        ROS_ERROR("Unknown pixel format");
        return;
      }
      image_msg_.header.stamp = image_stamp;
      cam_info_msg_.header.stamp = image_msg_.header.stamp;

      // publish the image
      cam_pub_.publish(image_msg_, cam_info_msg_, image_msg_.header.stamp);
    }

    image->Release();
  }

public:
  CameraPtr cam_ptr_;

private:
  sensor_msgs::Image image_msg_;
  sensor_msgs::CameraInfo cam_info_msg_;
  std::shared_ptr<DeviceEventHandlerImpl> device_event_handler_;
  image_transport::CameraPublisher cam_pub_;
  std::string cam_name_;
  ros::Time last_image_stamp_;
  bool exp_time_comp_flag_ = false;
};
#endif  // SPINNAKER_DRIVER_IMAGE_EVENT_HANDLER_IMPL_HPP_