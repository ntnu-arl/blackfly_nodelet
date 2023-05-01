// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef SPINNAKER_DRIVER_DEVICE_EVENT_HANDLER_HPP_
#define SPINNAKER_DRIVER_DEVICE_EVENT_HANDLER_HPP_

// C++
#include <mutex>

// ROS
#include <ros/ros.h>

// Spinnaker
#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class DeviceEventHandlerImpl : public DeviceEventHandler
{
public:
  DeviceEventHandlerImpl(CameraPtr cam_ptr)
  {
    // save the camera pointer
    cam_ptr_ = cam_ptr;
    // get the GENAPI node map
    INodeMap & node_map = cam_ptr_->GetNodeMap();
    try {
      CEnumerationPtr ptr_event_selector = node_map.GetNode("EventSelector");
      if (!IsAvailable(ptr_event_selector) || !IsReadable(ptr_event_selector)) {
        ROS_ERROR("Unable to retrieve event selector entries. Aborting");
        return;
      }
      NodeList_t entries;
      ptr_event_selector->GetEntries(entries);
      for (unsigned int i = 0; i < entries.size(); i++) {
        // Select entry on selector node
        CEnumEntryPtr ptr_enum_entry = entries.at(i);
        if (ptr_enum_entry->GetDisplayName() == "Exposure End") {
          if (!IsAvailable(ptr_enum_entry) || !IsReadable(ptr_enum_entry)) {
            // Skip if node fails
            ROS_WARN("Exposure end event is unavailable or unreadable");
            continue;
          }
          // The next line may be unnecessary - It seems to write the same value that was already
          // available?
          ptr_event_selector->SetIntValue(ptr_enum_entry->GetValue());
          // Retrieve event notification node (an enumeration node)
          CEnumerationPtr ptr_event_notification = node_map.GetNode("EventNotification");
          if (!IsAvailable(ptr_event_notification) || !IsWritable(ptr_event_notification)) {
            ROS_WARN("Event Notification is unavailable or unwritable");
            continue;
          }
          // Retrieve entry node to enable device event
          CEnumEntryPtr ptr_event_notification_on = ptr_event_notification->GetEntryByName("On");
          if (!IsAvailable(ptr_event_notification) || !IsReadable(ptr_event_notification)) {
            ROS_WARN("Event Notification is unavailable or unreadable");
            continue;
          }
          ptr_event_notification->SetIntValue(ptr_event_notification_on->GetValue());
        }
      }
    } catch (Spinnaker::Exception & e) {
      ROS_FATAL("Failed to configure device event handler");
    }
  }
  ~DeviceEventHandlerImpl() {}
  void OnDeviceEvent(gcstring event_name)
  {
    if (event_name == "EventExposureEnd") {
      // lock the mutex to prevent changes to member timestamp object
      timestamp_mutex_.lock();
      // get the now time as the end of the exposure
      last_frame_time_ = ros::Time::now();
      // unlock the mutex
      timestamp_mutex_.unlock();
    }
  }
  ros::Time getLastExposureEnd()
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    if (last_frame_time_.toSec() == 0.0) {
      return ros::Time(0, 0);
    } else {
      // get the last frame time
      ros::Time stamp = last_frame_time_;
      // assign the member timestamp to 0 to indicate it's already been reported
      last_frame_time_ = ros::Time(0, 0);
      // return the stamp
      return stamp;
    }
  }

private:
  // member mutex to lock the timestamp member when is set/get
  std::mutex timestamp_mutex_;
  // initialize the timestamp member to 0.0 to indicate it has not been set
  ros::Time last_frame_time_ = ros::Time(0, 0);
  // Camera pointer to spinnaker camera object (used to get the current exposure time)
  CameraPtr cam_ptr_;
};
#endif  // SPINNAKER_DRIVER_DEVICE_EVENT_HANDLER_HPP_
