// Copyright (c) 2022, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "spinnaker_driver/spinnaker_driver.hpp"

namespace spinnaker_driver
{
SpinnakerDriver::SpinnakerDriver(ros::NodeHandle & pnh)
{
  // Read parameters
  std::vector<std::string> camera_serials;
  pnh.getParam("camera_serial_nums", camera_serials);

  std::vector<std::string> camera_names;
  pnh.getParam("camera_names", camera_names);

  std::vector<std::string> camera_info_paths;
  pnh.getParam("camera_info_paths", camera_info_paths);

  std::vector<bool> mono_flags;
  pnh.getParam("mono_flags", mono_flags);

  std::vector<bool> is_triggered_flags;
  pnh.getParam("is_triggered_flags", is_triggered_flags);

  std::vector<float> trigger_delays;
  pnh.getParam("trigger_delays", trigger_delays);

  std::vector<float> fps;
  pnh.getParam("fps", fps);

  std::vector<bool> is_auto_exp_flags;
  pnh.getParam("is_auto_exp_flags", is_auto_exp_flags);

  std::vector<float> max_auto_exp;
  pnh.getParam("max_auto_exp", max_auto_exp);

  std::vector<float> min_auto_exp;
  pnh.getParam("min_auto_exp", min_auto_exp);

  std::vector<float> fixed_exp;
  pnh.getParam("fixed_exp", fixed_exp);

  std::vector<bool> auto_gain_flags;
  pnh.getParam("auto_gains", auto_gain_flags);

  std::vector<float> gains;
  pnh.getParam("gains", gains);

  std::vector<float> max_gains;
  pnh.getParam("max_gains", max_gains);

  std::vector<float> min_gains;
  pnh.getParam("min_gains", min_gains);

  std::vector<bool> enable_gamma;
  pnh.getParam("enable_gamma", enable_gamma);

  std::vector<float> gammas;
  pnh.getParam("gammas", gammas);

  std::vector<int> binnings;
  pnh.getParam("binnings", binnings);

  std::vector<int> binning_mode;
  pnh.getParam("binning_mode", binning_mode);

  std::vector<int> lighting_mode;
  pnh.getParam("lighting_mode", lighting_mode);

  std::vector<int> auto_exposure_priority;
  pnh.getParam("auto_exposure_priority", auto_exposure_priority);

  std::vector<bool> exp_comp_flags;
  pnh.getParam("exp_comp_flags", exp_comp_flags);

  std::vector<int> device_link_throughput_limits;
  pnh.getParam("device_link_throughput_limits", device_link_throughput_limits);

  bool enable_dyn_reconf;
  pnh.getParam("enable_dyn_reconf", enable_dyn_reconf);

  // Verify that the number of camera settings match the number of camera names
  int num_cameras_listed = camera_names.size();
  if (
    camera_serials.size() != num_cameras_listed || camera_info_paths.size() != num_cameras_listed ||
    mono_flags.size() != num_cameras_listed || is_triggered_flags.size() != num_cameras_listed ||
    trigger_delays.size() != num_cameras_listed || fps.size() != num_cameras_listed ||
    is_auto_exp_flags.size() != num_cameras_listed || max_auto_exp.size() != num_cameras_listed ||
    min_auto_exp.size() != num_cameras_listed || fixed_exp.size() != num_cameras_listed ||
    auto_gain_flags.size() != num_cameras_listed || gains.size() != num_cameras_listed ||
    max_gains.size() != num_cameras_listed || min_gains.size() != num_cameras_listed ||
    enable_gamma.size() != num_cameras_listed || gammas.size() != num_cameras_listed ||
    binnings.size() != num_cameras_listed || binning_mode.size() != num_cameras_listed ||
    lighting_mode.size() != num_cameras_listed ||
    auto_exposure_priority.size() != num_cameras_listed ||
    exp_comp_flags.size() != num_cameras_listed ||
    device_link_throughput_limits.size() != num_cameras_listed) {
    ROS_FATAL("Camera settings don't match number of camera names");
    ros::shutdown();
  }

  system_ = System::GetInstance();
  camera_list_ = system_->GetCameras();
  // Finish if there are no cameras
  if (camera_list_.GetSize() == 0) {
    teardown();
    ROS_FATAL("No spinnaker camera detected!");
    ros::shutdown();
  }

  for (int i = 0; i < camera_names.size(); i++) {
    ROS_DEBUG("Camera #%i", i);
    CameraPtr camera_ptr;
    try {
      ROS_DEBUG("Camera Serial : #%s", camera_serials[i].c_str());

      camera_ptr = camera_list_.GetBySerial(camera_serials[i]);
      if (!camera_ptr->IsValid()) {
        teardown();
        ROS_FATAL(
          "Failed to get camera pointer from spinnaker for camera serial: %s",
          camera_serials[i].c_str());
        ros::shutdown();
      }

      ROS_DEBUG("Camera Pointer Valid");
    } catch (const std::exception & e) {
      teardown();
      ROS_FATAL("Failed to find camera with serial : %s", camera_serials[i].c_str());
      ros::shutdown();
    }
    CameraSettings settings(
      camera_names[i], camera_info_paths[i], mono_flags[i], is_triggered_flags[i],
      trigger_delays[i], fps[i], is_auto_exp_flags[i], max_auto_exp[i], min_auto_exp[i],
      fixed_exp[i], auto_gain_flags[i], gains[i], max_gains[i], min_gains[i], enable_gamma[i],
      gammas[i], binnings[i], binning_mode[i], lighting_mode[i], auto_exposure_priority[i],
      exp_comp_flags[i], device_link_throughput_limits[i]);

    ROS_DEBUG("Created Camera Settings Object");

    cameras_.push_back(std::make_unique<Camera>(settings, camera_ptr, pnh));
    ROS_INFO(
      "Successfully launched camera : %s, Serial : %s", settings.cam_name.c_str(),
      camera_serials[i].c_str());
  }

  if (enable_dyn_reconf) {
    ROS_WARN_ONCE("Dynamic Reconfigure Triggered");
    dynamic_reconfigure::Server<spinnaker_driver::SpinnakerConfig> * dr_srv =
      new dynamic_reconfigure::Server<spinnaker_driver::SpinnakerConfig>(pnh);
    dynamic_reconfigure::Server<spinnaker_driver::SpinnakerConfig>::CallbackType dyn_rec_cb =
      boost::bind(&SpinnakerDriver::dynamicReconfigureCb, this, _1, _2);
    dr_srv->setCallback(dyn_rec_cb);
  }

  ROS_INFO("Successfully launched all cameras.");
}

SpinnakerDriver::~SpinnakerDriver() { teardown(); }

void SpinnakerDriver::teardown()
{
  cameras_.clear();
  camera_list_.Clear();
  system_->ReleaseInstance();
}

void SpinnakerDriver::dynamicReconfigureCb(
  spinnaker_driver::SpinnakerConfig & config, uint32_t level)
{
  std::cout << "Dynamic Reconfigure triggered" << std::endl;

  // fps
  camera_list_[config.cam_id]->AcquisitionFrameRate = config.fps;
  // gamma
  camera_list_[config.cam_id]->GammaEnable = config.enable_gamma;
  camera_list_[config.cam_id]->Gamma.SetValue(config.gamma);

  switch (config.exposure_auto) {
    case 0:
      camera_list_[config.cam_id]->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
      camera_list_[config.cam_id]->ExposureTime = config.exposure_time;
      break;
    case 1:
      camera_list_[config.cam_id]->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Once);
      break;
    default:
      camera_list_[config.cam_id]->ExposureAuto.SetValue(
        ExposureAutoEnums::ExposureAuto_Continuous);
      break;
  }
  switch (config.gain_auto) {
    case 0:
      camera_list_[config.cam_id]->GainAuto.SetValue(GainAutoEnums::GainAuto_Off);
      camera_list_[config.cam_id]->Gain = config.gain;
      break;
    case 1:
      camera_list_[config.cam_id]->GainAuto.SetValue(GainAutoEnums::GainAuto_Once);
      break;
    default:
      camera_list_[config.cam_id]->GainAuto.SetValue(GainAutoEnums::GainAuto_Continuous);
      break;
  }

  switch (config.lighting_mode) {
    case 1:
      camera_list_[config.cam_id]->AutoExposureLightingMode.SetValue(
        AutoExposureLightingModeEnums::AutoExposureLightingMode_Backlight);
      break;
    case 2:
      camera_list_[config.cam_id]->AutoExposureLightingMode.SetValue(
        AutoExposureLightingModeEnums::AutoExposureLightingMode_Frontlight);
      break;
    default:
      camera_list_[config.cam_id]->AutoExposureLightingMode.SetValue(
        AutoExposureLightingModeEnums::AutoExposureLightingMode_Normal);
      break;
  }
  switch (config.auto_exposure_priority) {
    case 1:
      camera_list_[config.cam_id]->AutoExposureControlPriority.SetValue(
        AutoExposureControlPriorityEnums::AutoExposureControlPriority_ExposureTime);
      break;
    default:
      camera_list_[config.cam_id]->AutoExposureControlPriority.SetValue(
        AutoExposureControlPriorityEnums::AutoExposureControlPriority_Gain);
      break;
  }

  if (config.acquisition_stop) {
    std::cout << "stop trigger" << std::endl;
    camera_list_[config.cam_id]->AcquisitionStop();
    camera_list_[config.cam_id]->TLParamsLocked = 0;
    camera_list_[config.cam_id]->BinningHorizontal = config.binning;
    camera_list_[config.cam_id]->BinningVertical = config.binning;

    switch (config.binning_mode) {
      case 0:
        camera_list_[config.cam_id]->BinningHorizontalMode.SetValue(
          BinningHorizontalModeEnums::BinningHorizontalMode_Average);
        camera_list_[config.cam_id]->BinningVerticalMode.SetValue(
          BinningVerticalModeEnums::BinningVerticalMode_Average);
        break;

      default:
        camera_list_[config.cam_id]->BinningHorizontalMode.SetValue(
          BinningHorizontalModeEnums::BinningHorizontalMode_Sum);
        camera_list_[config.cam_id]->BinningVerticalMode.SetValue(
          BinningVerticalModeEnums::BinningVerticalMode_Sum);
        break;
    }
  }
  if (config.acquisition_start) {
    std::cout << "start trigger" << std::endl;
    camera_list_[config.cam_id]->TLParamsLocked = 1;
    camera_list_[config.cam_id]->AcquisitionStart();
  }
}

}  // namespace spinnaker_driver
