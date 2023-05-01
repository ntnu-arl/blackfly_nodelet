// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

// C++
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

// Spinnaker
#include <Spinnaker.h>

// spinnaker_driver
#include "spinnaker_driver/image_event_handler_impl.hpp"
#include "spinnaker_driver/device_event_handler_impl.hpp"

using namespace Spinnaker;

namespace spinnaker_driver
{
struct CameraSettings
{
  CameraSettings()
  {
    cam_name = "None";
    cam_info_path = "None";
    mono = false;
    is_triggered = false;
    trigger_delay = 13.0;
    fps = 20.0;
    is_auto_exp = true;
    max_auto_exp_time = 30000.0;
    min_auto_exp_time = 50.0;
    fixed_exp_time = 5000.0;
    auto_gain = true;
    gain = 1.0;
    enable_gamma = true;
    gamma = 1.0;
    exp_comp_flag = false;
    device_link_throughput_limit = 40325200;
  }
  CameraSettings(std::string cam_name_p, std::string cam_info_path_p, bool mono_p,
                  bool is_triggered_p, float trigger_delay_p, float fps_p, bool is_auto_exp_p, float max_exp_p,
                  float min_exp_p, float fixed_exp_p, bool auto_gain_p, float gain_p,
                  float max_gain_p, float min_gain_p, bool enable_gamma_p, float gamma_p,
                  int binning_p, int binning_mode_p, int lighting_mode_p,
                  int auto_exposure_priority_p, bool exp_comp_flag_p, int device_link_throughput_limit_p)
  {
    cam_name = cam_name_p;
    cam_info_path = cam_info_path_p;
    mono = mono_p;
    is_triggered = is_triggered_p;
    trigger_delay = trigger_delay_p;
    fps = fps_p;
    is_auto_exp = is_auto_exp_p;
    max_auto_exp_time = max_exp_p;
    min_auto_exp_time = min_exp_p;
    fixed_exp_time = fixed_exp_p;
    auto_gain = auto_gain_p;
    gain = gain_p;
    max_gain = max_gain_p;
    min_gain = min_gain_p;
    enable_gamma = enable_gamma_p;
    gamma = gamma_p;
    binning = binning_p;
    binning_mode = binning_mode_p;
    lighting_mode = lighting_mode_p;
    auto_exposure_priority = auto_exposure_priority_p;
    exp_comp_flag = exp_comp_flag_p;
    device_link_throughput_limit = device_link_throughput_limit_p;
  }
  std::string cam_name;
  std::string cam_info_path;
  bool mono;
  bool is_triggered;
  float trigger_delay;
  float fps;
  bool is_auto_exp;
  float max_auto_exp_time;
  float min_auto_exp_time;
  float fixed_exp_time;
  bool auto_gain;
  float gain;
  float min_gain;
  float max_gain;
  bool enable_gamma;
  float gamma;
  int binning;
  int binning_mode;
  int lighting_mode;
  int auto_exposure_priority;
  bool exp_comp_flag;
  int device_link_throughput_limit;
};

class Camera
{
 public:
  Camera(CameraSettings settings, CameraPtr cam_ptr, ros::NodeHandle & pnh)
  {
    // save the camera pointer and the settings object
    settings_ = settings;
    cam_ptr_ = cam_ptr;

    // setup ros image transport
    it_ = std::make_unique<image_transport::ImageTransport>(pnh);
    cam_pub_ = it_->advertiseCamera(settings_.cam_name + "/image", 10);
    camera_info_manager::CameraInfoManager cam_info_mgr(pnh, settings_.cam_name, settings_.cam_info_path);
    cam_info_mgr.loadCameraInfo(settings_.cam_info_path);
    sensor_msgs::CameraInfo cam_info_msg  = cam_info_mgr.getCameraInfo();

    // setup the camera
    setup_camera();

    // create event handlers
    device_event_handler_ptr_ = std::make_shared<DeviceEventHandlerImpl>(cam_ptr_);
    image_event_handler_ptr_ = std::make_unique<ImageEventHandlerImpl>(
        settings_.cam_name, cam_ptr_, cam_pub_, cam_info_msg,
        device_event_handler_ptr_, settings_.exp_comp_flag);

    // register event handlers
    cam_ptr_->RegisterEventHandler(*device_event_handler_ptr_);
    cam_ptr_->RegisterEventHandler(*image_event_handler_ptr_);

    cam_ptr_->BeginAcquisition();
  }
  ~Camera()
  {
    if (cam_ptr_->IsValid())
    {
      cam_ptr_->EndAcquisition();
      cam_ptr_->UnregisterEventHandler(*image_event_handler_ptr_);
      cam_ptr_->UnregisterEventHandler(*device_event_handler_ptr_);
      image_event_handler_ptr_.reset();
      device_event_handler_ptr_.reset();
      cam_ptr_->DeInit();
      std::free(user_buffer_);
    }
  }
  // This function sets the internal buffersize of spinnaker -> By default it allocates memory based
  // on the frame rate of the camera. This may require very large memory when using multiple
  // cameras, and may cause additional issues.
  void set_buffer_size(unsigned int buff_size)
  {
    // Retrieve Stream Parameters device nodemap
    Spinnaker::GenApi::INodeMap &sNodeMap = cam_ptr_->GetTLStreamNodeMap();
    // Retrieve Buffer Handling Mode Information
    CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
    if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
    {
      ROS_ERROR("Unable to set Buffer Handling mode (node retrieval). Aborting...");
    }
    CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
    if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
    {
      ROS_ERROR("Unable to set Buffer Handling mode (Entry retrieval). Aborting...");
    }
    // Set stream buffer Count Mode to manual
    CEnumerationPtr ptrStreamBufferCountMode = sNodeMap.GetNode("StreamBufferCountMode");
    if (!IsAvailable(ptrStreamBufferCountMode) || !IsWritable(ptrStreamBufferCountMode))
    {
      ROS_ERROR("Unable to set Buffer Count Mode (node retrieval). Aborting...");
    }
    CEnumEntryPtr ptrStreamBufferCountModeManual =
        ptrStreamBufferCountMode->GetEntryByName("Manual");
    if (!IsAvailable(ptrStreamBufferCountModeManual) || !IsReadable(ptrStreamBufferCountModeManual))
    {
      ROS_ERROR("Unable to set Buffer Count Mode entry (Entry retrieval). Aborting...");
    }
    ptrStreamBufferCountMode->SetIntValue(ptrStreamBufferCountModeManual->GetValue());
    // Retrieve and modify Stream Buffer Count
    CIntegerPtr ptrBufferCount = sNodeMap.GetNode("StreamBufferCountManual");
    if (!IsAvailable(ptrBufferCount) || !IsWritable(ptrBufferCount))
    {
      ROS_ERROR("Unable to set Buffer Count (Integer node retrieval). Aborting...");
    }
    // Display Buffer Info
    ptrBufferCount->SetValue(buff_size);
  }
  void setup_camera()
  {
    try
    {
      // initialize the camera
      cam_ptr_->Init();

      cam_ptr_->AcquisitionStop();

      // Set up pixel format
      if (settings_.mono)
      {
        cam_ptr_->PixelFormat = PixelFormat_Mono8;
      }
      else
      {
        cam_ptr_->PixelFormat = PixelFormat_BGR8;
      }
      cam_ptr_->BinningVertical = settings_.binning;
      cam_ptr_->BinningHorizontal = settings_.binning;

      cam_ptr_->BinningHorizontal = settings_.binning;

      // set binning type 0=Average, 1=Sum
      if (settings_.binning_mode == 0)
      {
        cam_ptr_->BinningHorizontalMode.SetValue(
            BinningHorizontalModeEnums::BinningHorizontalMode_Average);
        cam_ptr_->BinningVerticalMode.SetValue(
            BinningVerticalModeEnums::BinningVerticalMode_Average);
      }
      else if (settings_.binning_mode == 1)
      {
        cam_ptr_->BinningHorizontalMode.SetValue(
            BinningHorizontalModeEnums::BinningHorizontalMode_Sum);
        cam_ptr_->BinningVerticalMode.SetValue(BinningVerticalModeEnums::BinningVerticalMode_Sum);
      }

      // set lighting type 0=Normal, 1=Backlight, 2=Frontlight
      if (settings_.lighting_mode == 1)
      {
        cam_ptr_->AutoExposureLightingMode.SetValue(
            AutoExposureLightingModeEnums::AutoExposureLightingMode_Backlight);
      }
      else if (settings_.lighting_mode == 2)
      {
        cam_ptr_->AutoExposureLightingMode.SetValue(
            AutoExposureLightingModeEnums::AutoExposureLightingMode_Frontlight);
      }

      // set acquisition mode, Continuous instead of single frame or burst modes
      cam_ptr_->AcquisitionMode = AcquisitionMode_Continuous;

      // setup exposure
      if (settings_.is_auto_exp)
      {
        cam_ptr_->ExposureAuto = ExposureAuto_Continuous;
        cam_ptr_->AutoExposureExposureTimeUpperLimit = settings_.max_auto_exp_time;
        cam_ptr_->AutoExposureExposureTimeLowerLimit = settings_.min_auto_exp_time;
      }
      else
      {
        cam_ptr_->ExposureAuto = ExposureAuto_Off;
        cam_ptr_->ExposureTime = settings_.fixed_exp_time;
      }
      // setup gain
      cam_ptr_->GainAuto = GainAuto_Off;
      if (settings_.auto_gain)
      {
        cam_ptr_->GainAuto = GainAuto_Continuous;
        cam_ptr_->AutoExposureGainUpperLimit = settings_.max_gain;
        cam_ptr_->AutoExposureGainLowerLimit = settings_.min_gain;
      }
      else
      {
        cam_ptr_->Gain.SetValue(settings_.gain);
      }
      // setup gamma
      if (settings_.enable_gamma)
      {
        cam_ptr_->GammaEnable = true;
        cam_ptr_->Gamma.SetValue(settings_.gamma);
      }
      else
      {
        cam_ptr_->GammaEnable = false;
      }
      // setup trigger parameters
      if (settings_.is_triggered)
      {
        cam_ptr_->TriggerMode = TriggerMode_Off;
        cam_ptr_->TriggerSource = TriggerSource_Line0;
        // cam_ptr_->TriggerSource = TriggerSource_Line1;
        // cam_ptr_->TriggerSource = TriggerSource_Line2;
        // cam_ptr_->TriggerSource = TriggerSource_Line2;
        // cam_ptr_->TriggerSource = TriggerSource_Counter0End;
        cam_ptr_->TriggerActivation = TriggerActivation_RisingEdge;
        cam_ptr_->TriggerMode = TriggerMode_On;
        cam_ptr_->AcquisitionFrameRateEnable = false;
        // cam_ptr_->Counter = ;
        cam_ptr_->TriggerDelay.SetValue(settings_.trigger_delay);
      }
      else
      {
        cam_ptr_->TriggerMode = TriggerMode_Off;
        cam_ptr_->AcquisitionFrameRateEnable = true;
        cam_ptr_->AcquisitionFrameRate = settings_.fps;
      }
      cam_ptr_->ExposureMode = ExposureMode_Timed;
      set_buffer_size(5);

      // Device Link Throughput Limit setting
      CIntegerPtr ptrDeviceLinkThroughputLimit = cam_ptr_->GetNodeMap().GetNode("DeviceLinkThroughputLimit");
      if (!IsAvailable(ptrDeviceLinkThroughputLimit) || !IsWritable(ptrDeviceLinkThroughputLimit))
      {
          std::cout << "Unable to set device link throughput limit (node retrieval; camera " << settings_.cam_name << "). Aborting..."
                << std::endl;
      }
      ptrDeviceLinkThroughputLimit->SetValue(settings_.device_link_throughput_limit);
    }
    catch (Spinnaker::Exception &ex)
    {
      ROS_ERROR("ERROR SETTING CAMERA SETTINGS!!!");
      std::cout << "Error: " << ex.what() << std::endl;
      std::cout << "Error code " << ex.GetError() << " raised in function " << ex.GetFunctionName()
                << " at line " << ex.GetLineNumber() << "." << std::endl;
    }
  }

 private:
  size_t total_size_ = sizeof(int8_t) * 1024;
  void *user_buffer_ = malloc(total_size_);
  // void* buffer = nullptr;
  CameraPtr cam_ptr_;
  CameraSettings settings_;
  std::shared_ptr<DeviceEventHandlerImpl> device_event_handler_ptr_;
  std::unique_ptr<ImageEventHandlerImpl> image_event_handler_ptr_;
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher cam_pub_;
};
}  // namespace spinnaker_driver