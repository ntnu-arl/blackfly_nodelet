#include <vector>
#include <string>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "Spinnaker.h"
#include "image_event_handler.h"
#include "device_event_handler.h"

using namespace Spinnaker;

struct camera_settings
{
	camera_settings()
	{
		cam_name = "None";
		cam_info_path = "None";
		mono = false;
		is_triggered = false;
		fps = 20.0;
		is_auto_exp = true;
		max_auto_exp_time = 30000.0;
		min_auto_exp_time = 500.0;
		fixed_exp_time = 5000.0;
		auto_gain = true;
		gain = 1.0;
		enable_gamma = true;
		gamma = 1.0;
	}
	camera_settings(std::string cam_name_p, std::string cam_info_path_p, bool mono_p, bool is_triggered_p, float fps_p,
					bool is_auto_exp_p, float max_exp_p, float min_exp_p, float fixed_exp_p,
					bool auto_gain_p, float gain_p, float max_gain_p, float min_gain_p, bool enable_gamma_p, float gamma_p)
	{
		cam_name = cam_name_p;
		cam_info_path = cam_info_path_p;
		mono = mono_p;
		is_triggered = is_triggered_p;
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

	}
	std::string cam_name;
	std::string cam_info_path;
	bool mono;
	bool is_triggered;
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
};


class blackfly_camera
{
	public:
		blackfly_camera(camera_settings settings, CameraPtr cam_ptr, image_transport::ImageTransport* image_transport_ptr, ros::NodeHandle* pnh)
		{
			ROS_INFO("Blackfly Constructor");
			m_cam_ptr = cam_ptr;
			m_cam_settings = settings;

			m_image_transport_ptr = image_transport_ptr;
			m_cam_pub = image_transport_ptr->advertiseCamera(m_cam_settings.cam_name, 10);
			m_cam_info_mgr_ptr = boost::make_shared<camera_info_manager::CameraInfoManager>(*pnh, m_cam_settings.cam_name, m_cam_settings.cam_info_path);
			m_cam_info_mgr_ptr->loadCameraInfo(m_cam_settings.cam_info_path);

			ROS_INFO("Initing Camera");
			setup_camera();

			ROS_INFO("Creating Event Handlers");
			m_device_event_handler_ptr = new DeviceEventHandler(m_cam_ptr);
			m_image_event_handler_ptr = new ImageEventHandler(m_cam_settings.cam_name, m_cam_ptr, &m_cam_pub, m_cam_info_mgr_ptr, m_device_event_handler_ptr);
		
			ROS_INFO("Registering Event Handlers");
			m_cam_ptr->RegisterEvent(*m_device_event_handler_ptr);
			m_cam_ptr->RegisterEvent(*m_image_event_handler_ptr);
			ROS_INFO("Constructor Complete");
			m_cam_ptr->BeginAcquisition();
		}
		~blackfly_camera()
		{
			m_cam_ptr->EndAcquisition();
			m_cam_ptr->UnregisterEvent(*m_image_event_handler_ptr);
			m_cam_ptr->UnregisterEvent(*m_device_event_handler_ptr);
			delete m_image_event_handler_ptr;
			delete m_device_event_handler_ptr;
			m_cam_ptr->DeInit();
		}
		void setup_camera()
		{
			try
			{
				// initialize the camera
				m_cam_ptr->Init();
				// stop acquisition
				m_cam_ptr->AcquisitionStop();

				// Set up pixel format
				if(m_cam_settings.mono)
				{
					m_cam_ptr->PixelFormat = PixelFormat_Mono8;
				}
				else
				{
					m_cam_ptr->PixelFormat = PixelFormat_BGR8;
				}

				// set acquisition mode
				m_cam_ptr->AcquisitionMode = AcquisitionMode_Continuous;

				// setup trigger parameters
				if(m_cam_settings.is_triggered)
				{
					m_cam_ptr->TriggerMode = TriggerMode_Off;
					m_cam_ptr->TriggerSource = TriggerSource_Line0;
					// m_cam_ptr->TriggerSource = TriggerSource_Line1;
					// m_cam_ptr->TriggerSource = TriggerSource_Line2;
					// m_cam_ptr->TriggerSource = TriggerSource_Line2;
					m_cam_ptr->TriggerMode = TriggerMode_On;
				}
				else
				{
					m_cam_ptr->TriggerMode = TriggerMode_Off;
					m_cam_ptr->AcquisitionFrameRateEnable = true;
					m_cam_ptr->AcquisitionFrameRate = m_cam_settings.fps;
				}
				m_cam_ptr->ExposureMode = ExposureMode_Timed;

				if(m_cam_settings.is_auto_exp)
				{
					m_cam_ptr->AutoExposureExposureTimeUpperLimit = m_cam_settings.max_auto_exp_time;
					m_cam_ptr->AutoExposureExposureTimeLowerLimit = m_cam_settings.min_auto_exp_time;
					m_cam_ptr->ExposureAuto = ExposureAuto_Continuous;
				}
				else
				{
					m_cam_ptr->ExposureAuto = ExposureAuto_Off;
					m_cam_ptr->ExposureTime = m_cam_settings.fixed_exp_time;
				}
				m_cam_ptr->GainAuto = GainAuto_Off;
				if(m_cam_settings.auto_gain)
				{
					m_cam_ptr->AutoExposureGainUpperLimit = m_cam_settings.max_gain;
					m_cam_ptr->AutoExposureGainLowerLimit = m_cam_settings.min_gain;
					m_cam_ptr->GainAuto = GainAuto_Continuous;
				}
				else
				{
					m_cam_ptr->Gain.SetValue(m_cam_settings.gain);
				}
				if(m_cam_settings.enable_gamma)
				{
					m_cam_ptr->GammaEnable = true;
					m_cam_ptr->Gamma.SetValue(m_cam_settings.gamma);
				}
				else
				{
					m_cam_ptr->GammaEnable = false;
				}
			}
			catch (Spinnaker::Exception & ex)
			{
				ROS_ERROR("ERROR SETTING CAMERA SETTINGS!!!");
				std::cout << "Error: " << ex.what() << std::endl;
				std::cout << "Error code " << ex.GetError() << " raised in function " << ex.GetFunctionName() << " at line " << ex.GetLineNumber() << "." << std::endl;
			}
		}
	private:
		CameraPtr m_cam_ptr;
		camera_settings m_cam_settings;
		ImageEventHandler *m_image_event_handler_ptr;
		DeviceEventHandler *m_device_event_handler_ptr;
		image_transport::ImageTransport* m_image_transport_ptr;
		image_transport::CameraPublisher m_cam_pub;
		boost::shared_ptr<camera_info_manager::CameraInfoManager>m_cam_info_mgr_ptr;

};