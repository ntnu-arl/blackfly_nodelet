#include "../include/blackfly_nodelet.h"

// Nodelet stuff
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(blackfly::blackfly_nodelet, nodelet::Nodelet)

namespace blackfly
{
	blackfly_nodelet::~blackfly_nodelet()
	{
		// Release system
		camList.Clear();
		system->ReleaseInstance();	
		delete image_transport_ptr;
	}
	void blackfly_nodelet::onInit()
	{
		ros::NodeHandle& nh = getMTNodeHandle();
		ros::NodeHandle& pnh = getMTPrivateNodeHandle();

		image_transport_ptr = new image_transport::ImageTransport(pnh);

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



		system = System::GetInstance();
		camList = system->GetCameras();
		numCameras = camList.GetSize();

		// Finish if there are no cameras
		if (numCameras == 0)
		{
			camList.Clear();
			system->ReleaseInstance();
			ROS_FATAL("No spinnaker camera detected!");
			return;
		}

		for(int i = 0; i < camera_names.size(); i++)
		{
			CameraPtr cam_ptr;
			try
			{
				cam_ptr = camList.GetBySerial(camera_serials[i]);
				if(!cam_ptr->IsValid())
				{
					ROS_FATAL("Failed to get camera pointer from spinnaker");
					ros::shutdown();
				}
			}
			catch(const std::exception& e)
			{
				ROS_FATAL("Failed to find camera with serial : %s", camera_serials[i].c_str());
				ros::shutdown();
			}
			camera_settings settings(camera_names[i], camera_info_paths[i], mono_flags[i], 
							is_triggered_flags[i], fps[i], is_auto_exp_flags[i], max_auto_exp[i], min_auto_exp[i], fixed_exp[i],
							auto_gain_flags[i],gains[i], max_gains[i], min_gains[i],  enable_gamma[i], gammas[i]);

			blackfly_camera *blackfly_ptr = new blackfly_camera(settings, cam_ptr, image_transport_ptr, &pnh);

			
		}
	}
} // end namespace blackfly