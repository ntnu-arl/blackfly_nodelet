#include "../include/blackfly_nodelet.h"

// Nodelet stuff
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(blackfly::blackfly_nodelet, nodelet::Nodelet)

typedef void * (*THREADFUNCPTR)(void *);

namespace blackfly
{
	blackfly_nodelet::~blackfly_nodelet()
	{
		cam_ptr->EndAcquisition();
		cam_ptr->UnregisterEvent(*device_event_handler);
		cam_ptr->UnregisterEvent(*image_event_handler);
		cam_ptr->DeInit();
		// Release system
		camList.Clear();
		system->ReleaseInstance();	
	}
	void blackfly_nodelet::onInit()
	{
		ros::NodeHandle& nh = getMTNodeHandle();
		ros::NodeHandle& pnh = getMTPrivateNodeHandle();

		image_transport_ptr = new image_transport::ImageTransport(pnh);

		std::string camera_serial = "";
		pnh.getParam("camera_serial_num", camera_serial);

		std::string camera_name = "";
		pnh.getParam("camera_name", camera_name);

		ROS_INFO("Blackfly Nodelet Starting, Camera Name : %s, Camera Serial : %s", camera_name.c_str(), camera_serial.c_str());

		std::string camera_info_path = "package://blackfly/camera_info/cam_1.info";
		pnh.getParam("camera_info_path", camera_info_path);

		ROS_INFO("Blackfly Nodelet Starting, Camera camera_info_path : %s", camera_info_path.c_str());

		system = System::GetInstance();

		// get a reference to the camera list detected by spinnaker
		camList = system->GetCameras();

		// get the number of cameras detected by spinnaker
		numCameras = camList.GetSize();

		ROS_INFO("Number of cameras detected: %i", numCameras);

		// Finish if there are no cameras
		if (numCameras == 0)
		{
			// Clear camera list before releasing system
			camList.Clear();
			// Release system
			system->ReleaseInstance();
			ROS_FATAL("No spinnaker camera detected!");
			return;
		}
		try
		{
			cam_ptr = camList.GetBySerial(camera_serial);
		}
		catch(const std::exception& e)
		{
			ROS_ERROR("Failed to get camera, error = %s", e.what());
			return;
		}

		ROS_INFO("GOT CAMERA");
		//std::string cam_serial_num = cam_ptr->DeviceSerialNumber.ToString().c_str();
		image_transport::CameraPublisher cam_pub = image_transport_ptr->advertiseCamera("CAM_0", 10);
		c_info_mgr_ptr = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh, camera_name, camera_info_path);
		c_info_mgr_ptr->loadCameraInfo(camera_info_path);
		
		ROS_INFO("INIT CAMERA");
		cam_ptr->Init();
		ROS_INFO("End Acquisition");
		cam_ptr->AcquisitionStop();
		ROS_INFO("GET NODEMAP");
		INodeMap &cam_node_map = cam_ptr->GetNodeMap();
		//ROS_INFO("ENABLE CHUNK DATA");
		//enable_chunk_data(cam_node_map);
		ROS_INFO("SET Acquisition Mode");
		cam_ptr->AcquisitionMode = AcquisitionMode_Continuous;
		cam_ptr->AcquisitionFrameRateEnable = true;
		cam_ptr->AcquisitionFrameRate = 20.0;
		cam_ptr->ExposureMode = ExposureMode_Timed;
		cam_ptr->ExposureAuto = ExposureAuto_Continuous;
		//ROS_INFO("Config Sequencer");
		//configure_sequencer(cam_node_map);
		ROS_INFO("Create Event Handler");
		device_event_handler = new DeviceEventHandler(cam_node_map);
		image_event_handler = new ImageEventHandler(camera_name, cam_ptr, &cam_pub, c_info_mgr_ptr, device_event_handler);
		ROS_INFO("Register Event Handler");
		cam_ptr->RegisterEvent(*image_event_handler);
		cam_ptr->RegisterEvent(*device_event_handler);
		ROS_INFO("Start Acquisition");
		cam_ptr->BeginAcquisition();
	}
	void blackfly_nodelet::configure_sequencer(INodeMap &cam_node_map)
	{
		// FIRST, DISABLE SEQUENCER MODE

					CEnumerationPtr ptrSequencerConfigurationValid = cam_node_map.GetNode("SequencerConfigurationValid");
					if (!IsAvailable(ptrSequencerConfigurationValid) || !IsReadable(ptrSequencerConfigurationValid))
					{
						ROS_ERROR("Failed to retrieve node: SequencerConfigurationValid");
						return;
					}
					CEnumEntryPtr ptrSequencerConfigurationValidYes = ptrSequencerConfigurationValid->GetEntryByName("Yes");
					if (!IsAvailable(ptrSequencerConfigurationValidYes) || !IsReadable(ptrSequencerConfigurationValidYes))
					{
						ROS_ERROR("Failed to retrieve node: SequencerConfigurationValid 'Yes'");
						return;
					}
					// If valid, disable sequencer mode; otherwise, do nothing
					CEnumerationPtr ptrSequencerMode = cam_node_map.GetNode("SequencerMode");
					if (ptrSequencerConfigurationValid->GetCurrentEntry()->GetValue() == ptrSequencerConfigurationValidYes->GetValue())
					{
						if (!IsAvailable(ptrSequencerMode) || !IsWritable(ptrSequencerMode))
						{
							ROS_ERROR("Failed to set node: SequencerMode");
							return;
						}

						CEnumEntryPtr ptrSequencerModeOff = ptrSequencerMode->GetEntryByName("Off");
						if (!IsAvailable(ptrSequencerModeOff) || !IsReadable(ptrSequencerModeOff))
						{
							ROS_ERROR("Failed to set node: SequencerMode Off");
							return;
						}

						ptrSequencerMode->SetIntValue(static_cast<int64_t>(ptrSequencerModeOff->GetValue()));
					}
					ROS_INFO("Sequencer Mode Disabled");
		// SECOND, DISABLE AUTO EXPOSURE
					CEnumerationPtr ptrExposureAuto = cam_node_map.GetNode("ExposureAuto");
					if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
					{
						ROS_ERROR("Failed to get node: ExposureAuto");
						return;
					}

					CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
					if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
					{
						ROS_ERROR("Failed to get node: ExposureAuto Off");
						return;
					}

					ptrExposureAuto->SetIntValue(static_cast<int64_t>(ptrExposureAutoOff->GetValue()));
					ROS_INFO("Auto Exposure Set to Off");
		// THIRD, DISABLE AUTO GAIN
					CEnumerationPtr ptrGainAuto = cam_node_map.GetNode("GainAuto");
					if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
					{
						ROS_ERROR("Failed to get node: GainAuto");
						return;
					}

					CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
					if (!IsAvailable(ptrGainAutoOff) || !IsReadable(ptrGainAutoOff))
					{
						ROS_ERROR("Failed to get node: GainAuto Off");
						return;
					}

					ptrGainAuto->SetIntValue(static_cast<int64_t>(ptrGainAutoOff->GetValue()));

					ROS_INFO("Automatic gain disabled");
		// FOURTH, Turn Configuration Mode On
					CEnumerationPtr ptrSequencerConfigurationMode = cam_node_map.GetNode("SequencerConfigurationMode");
					if (!IsAvailable(ptrSequencerConfigurationMode) || !IsWritable(ptrSequencerConfigurationMode))
					{
						ROS_ERROR("Failed to get node: SequencerConfigurationMode");
						return;
					}
					CEnumEntryPtr ptrSequencerConfigurationModeOn = ptrSequencerConfigurationMode->GetEntryByName("On");
					if (!IsAvailable(ptrSequencerConfigurationModeOn) || !IsReadable(ptrSequencerConfigurationModeOn))
					{
						ROS_ERROR("Failed to get node: SequencerConfigurationMode On");
						return;
					}

					ptrSequencerConfigurationMode->SetIntValue(static_cast<int64_t>(ptrSequencerConfigurationModeOn->GetValue()));
					ROS_INFO("Sequencer configuration mode enabled");
		// FIFTH, SET SOME STATES
					unsigned int num_seqs = 5;
					double exposure_settings[] 	= {800.0, 	1500.0, 3000.0, 6000.0, 15000.0, 	45000.0	};
					double gain_settings[] 		= {1.0,		1.0,	1.0, 	1.0,	1.0,		1.0		};
					for(int i  = 0; i < num_seqs; i++)
					{
						set_sequence_state(cam_node_map, i, exposure_settings[i], gain_settings[i], num_seqs-1);
					}
		// SIXTH, TURN OFF CONFIG MODE
					ptrSequencerConfigurationMode = cam_node_map.GetNode("SequencerConfigurationMode");
					if (!IsAvailable(ptrSequencerConfigurationMode) || !IsWritable(ptrSequencerConfigurationMode))
					{
						ROS_ERROR("Failed to Set : node, SequencerConfigurationMode");
						return;
					}

					CEnumEntryPtr ptrSequencerConfigurationModeOff = ptrSequencerConfigurationMode->GetEntryByName("Off");
					if (!IsAvailable(ptrSequencerConfigurationModeOff) || !IsReadable(ptrSequencerConfigurationModeOff))
					{
						ROS_ERROR("Failed to Set : entry, SequencerConfigurationMode Off");
						return;
					}

					ptrSequencerConfigurationMode->SetIntValue(static_cast<int64_t>(ptrSequencerConfigurationModeOff->GetValue()));

					ROS_INFO("Sequencer configuration mode disabled");
		// SEVENTH Turn on Sequencer Mode
					ptrSequencerMode = cam_node_map.GetNode("SequencerMode");
					if (!IsAvailable(ptrSequencerMode) || !IsWritable(ptrSequencerMode))
					{
						ROS_ERROR("Failed to Set : node, SequencerMode");
						return;
					}

					CEnumEntryPtr ptrSequencerModeOn = ptrSequencerMode->GetEntryByName("On");
					if (!IsAvailable(ptrSequencerModeOn) || !IsReadable(ptrSequencerModeOn))
					{
						ROS_ERROR("Failed to Set : entry, SequencerMode On");
						return;
					}

					ptrSequencerMode->SetIntValue(static_cast<int64_t>(ptrSequencerModeOn->GetValue()));

					ROS_INFO("Sequencer mode enabled");
		// EIGHTH Check Sequencer Config
				ptrSequencerConfigurationValid = cam_node_map.GetNode("SequencerConfigurationValid");
				if (!IsAvailable(ptrSequencerConfigurationValid) || !IsReadable(ptrSequencerConfigurationValid))
				{
					ROS_ERROR("Failed to Set : node, SequencerConfigurationValid");
					return;
				}

				ptrSequencerConfigurationValidYes = ptrSequencerConfigurationValid->GetEntryByName("Yes");
				if (!IsAvailable(ptrSequencerConfigurationValidYes) || !IsReadable(ptrSequencerConfigurationValidYes))
				{
					ROS_ERROR("Failed to Set : entry, SequencerConfigurationValid Yes");
					return;
				}
				if (ptrSequencerConfigurationValid->GetCurrentEntry()->GetValue() != ptrSequencerConfigurationValidYes->GetValue())
				{
					ROS_ERROR("Sequencer configuration not valid. Aborting");
					return;
				}

				ROS_INFO("Sequencer configuration valid");		
	}
	void blackfly_nodelet::blackfly_nodelet::set_sequence_state(INodeMap &cam_node_map, unsigned int seq_num, double exposure_time, double gain_setting, unsigned int final_seq_index)
	{
		try
		{
		// SETUP EXPOSURE TIME
			CIntegerPtr ptrSequencerSetSelector = cam_node_map.GetNode("SequencerSetSelector");
			if (!IsAvailable(ptrSequencerSetSelector) || !IsWritable(ptrSequencerSetSelector))
			{
				ROS_ERROR("Unable to set current state. Aborting");
				return;
			}
			ptrSequencerSetSelector->SetValue(seq_num);
			ROS_INFO("Setting sequencer state %i",seq_num);
			CFloatPtr ptrExposureTime = cam_node_map.GetNode("ExposureTime");
			if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
			{
				ROS_INFO("Unable to set exposure time (node retrieval). Aborting");
				return;
			}
			ptrExposureTime->SetValue(exposure_time);
			ROS_INFO("Exposure time set to %f uSec", ptrExposureTime->GetValue());
		// SETUP GAIN
			CFloatPtr ptrGain = cam_node_map.GetNode("Gain");
			if (!IsAvailable(ptrGain) || !IsWritable(ptrGain))
			{
				ROS_ERROR("Unable to set gain (node retrieval). Aborting");
				return;
			}

			ptrGain->SetValue(gain_setting);
			ROS_INFO("\tGain set to %f dB", ptrGain->GetValue());
		// SETUP the State's trigger source
			CEnumerationPtr ptrSequencerTriggerSource = cam_node_map.GetNode("SequencerTriggerSource");
			if (!IsAvailable(ptrSequencerTriggerSource) || !IsWritable(ptrSequencerTriggerSource))
			{
				ROS_ERROR("Unable to set trigger source (node retrieval). Aborting");
				return;
			}

			CEnumEntryPtr ptrSequencerTriggerSourceFrameStart = ptrSequencerTriggerSource->GetEntryByName("FrameStart");
			if (!IsAvailable(ptrSequencerTriggerSourceFrameStart) || !IsReadable(ptrSequencerTriggerSourceFrameStart))
			{
				ROS_ERROR("Unable to set trigger source (enum entry retrieval). Aborting");
				return;
			}
			ptrSequencerTriggerSource->SetIntValue(static_cast<int64_t>(ptrSequencerTriggerSourceFrameStart->GetValue()));
			ROS_INFO("\tTrigger source set to start of frame");
		// SETUP the next state in the sequence
			CIntegerPtr ptrSequencerSetNext = cam_node_map.GetNode("SequencerSetNext");
			if (!IsAvailable(ptrSequencerSetNext) || !IsWritable(ptrSequencerSetNext))
			{
				ROS_ERROR("Unable to select next state. Aborting");
				return;
			}
			if (seq_num == final_seq_index)
			{
				ptrSequencerSetNext->SetValue(0);
			}
			else
			{
				ptrSequencerSetNext->SetValue(seq_num + 1);
			}
			ROS_INFO("\tNext state set to %i", ptrSequencerSetNext->GetValue());
		// SAVE the STATE
			CCommandPtr ptrSequencerSetSave = cam_node_map.GetNode("SequencerSetSave");
			if (!IsAvailable(ptrSequencerSetSave) || !IsWritable(ptrSequencerSetSave))
			{
				ROS_ERROR("Unable to save state. Aborting");
				return;
			}
			ptrSequencerSetSave->Execute();
			ROS_INFO("Current state saved");
		}
		catch(Spinnaker::Exception &e)
		{
			ROS_ERROR("FAILED SETTING SEQUENCE #%i : %s", seq_num, e.what());
		}
	}
	void blackfly_nodelet::enable_chunk_data(INodeMap &cam_node_map)
	{
		CBooleanPtr ptrChunkModeActive = cam_node_map.GetNode("ChunkModeActive");
		if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive))
		{
			ROS_ERROR("Unable to activate chunk mode. Aborting");
			return;
		}
		ptrChunkModeActive->SetValue(true);
		ROS_INFO("Chunk mode activated");
		NodeList_t entries;

		// Retrieve the selector node
		CEnumerationPtr ptrChunkSelector = cam_node_map.GetNode("ChunkSelector");

		if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
		{
			ROS_ERROR("Unable to retrieve chunk selector. Aborting");
			return;
		}

		// Retrieve entries
		ptrChunkSelector->GetEntries(entries);

		ROS_INFO("Enabling entries");

		for (size_t i = 0; i < entries.size(); i++)
		{
			// Select entry to be enabled
			CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);

			// Go to next node if problem occurs
			if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry))
			{
				continue;
			}

			ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

			ROS_INFO("\t %s:",ptrChunkSelectorEntry->GetSymbolic().c_str());

			// Retrieve corresponding boolean
			CBooleanPtr ptrChunkEnable = cam_node_map.GetNode("ChunkEnable");

			// Enable the boolean, thus enabling the corresponding chunk data
			if (!IsAvailable(ptrChunkEnable))
			{
				ROS_ERROR("\t \t not available");
			}
			else if (ptrChunkEnable->GetValue())
			{
				ROS_INFO("\t \t enabled");
			}
			else if (IsWritable(ptrChunkEnable))
			{
				ptrChunkEnable->SetValue(true);
				ROS_INFO("\t \t enabled");
			}
			else
			{
				ROS_ERROR("\t \t not writable");
			}
		}
	}
	void* blackfly_nodelet::get_images_thread(void* arg)
	{
		ROS_INFO("STARTING GET IMAGES THREAD");
		cam_ptr->Init();
		INodeMap &cam_node_map = cam_ptr->GetNodeMap();

		enable_chunk_data(cam_node_map);

		cam_ptr->AcquisitionMode = AcquisitionMode_Continuous;

		configure_sequencer(cam_node_map);
		cam_ptr->BeginAcquisition();
		for(int i = 0; i < 20; i++)
		{
			ImagePtr image_ptr = cam_ptr->GetNextImage();
			if (image_ptr->IsIncomplete())
			{
				ROS_ERROR("Image incomplete with image status : %s", image_ptr->GetImageStatus());
			}
			else
			{
				ROS_INFO("GOT IMAGE!");
				ChunkData image_chunk_data = image_ptr->GetChunkData();
				ROS_INFO("FRAME ID  : %i", image_chunk_data.GetFrameID());
				ROS_INFO("SEQ   ID  : %i", image_chunk_data.GetSequencerSetActive());
				ROS_INFO("Exposure  : %f", image_chunk_data.GetExposureTime());
				ROS_INFO("Gain      : %f", image_chunk_data.GetGain());
				ROS_INFO("Timestamp : %i", image_chunk_data.GetTimestamp());
				ROS_INFO("Now Stamp : %f", ros::Time::now().toSec());
				image_ptr->Release();
			}
		}
	}
} // end namespace blackfly
