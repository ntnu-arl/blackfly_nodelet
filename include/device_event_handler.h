#ifndef DEV_EVENT_HANDLER_
#define DEV_EVENT_HANDLER_
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream> 
#include <ros/ros.h>
#include <deque>
#include <mutex>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class DeviceEventHandler : public DeviceEvent
{
	public:
		DeviceEventHandler(CameraPtr cam_ptr)
		{
			// save the camera pointer
			m_cam_ptr = cam_ptr;
			// get the GENAPI node map
			INodeMap &node_map = m_cam_ptr->GetNodeMap();
			try
			{
				CEnumerationPtr ptrEventSelector = node_map.GetNode("EventSelector");
				if (!IsAvailable(ptrEventSelector) || !IsReadable(ptrEventSelector))
				{
					ROS_ERROR("Unable to retrieve event selector entries. Aborting");
					return;
				}
				NodeList_t entries;
				ptrEventSelector->GetEntries(entries);
				for (unsigned int i = 0; i < entries.size(); i++)
				{
					// Select entry on selector node
					CEnumEntryPtr ptrEnumEntry = entries.at(i);
					if(ptrEnumEntry->GetDisplayName() == "Exposure End")
					{
						if (!IsAvailable(ptrEnumEntry) || !IsReadable(ptrEnumEntry))
						{
							// Skip if node fails
							continue;
						}
						ptrEventSelector->SetIntValue(ptrEnumEntry->GetValue());
						// Retrieve event notification node (an enumeration node)
						CEnumerationPtr ptrEventNotification = node_map.GetNode("EventNotification");
						if (!IsAvailable(ptrEventNotification) || !IsWritable(ptrEventNotification))
						{
							continue;
						}
						// Retrieve entry node to enable device event
						CEnumEntryPtr ptrEventNotificationOn = ptrEventNotification->GetEntryByName("On");
						if (!IsAvailable(ptrEventNotification) || !IsReadable(ptrEventNotification))
						{
							continue;
						}
						ptrEventNotification->SetIntValue(ptrEventNotificationOn->GetValue());
					}
					if(ptrEnumEntry->GetDisplayName() == "Exposure Start")
					{
						if (!IsAvailable(ptrEnumEntry) || !IsReadable(ptrEnumEntry))
						{
							// Skip if node fails
							continue;
						}
						ptrEventSelector->SetIntValue(ptrEnumEntry->GetValue());
						// Retrieve event notification node (an enumeration node)
						CEnumerationPtr ptrEventNotification = node_map.GetNode("EventNotification");
						if (!IsAvailable(ptrEventNotification) || !IsWritable(ptrEventNotification))
						{
							continue;
						}
						// Retrieve entry node to enable device event
						CEnumEntryPtr ptrEventNotificationOn = ptrEventNotification->GetEntryByName("On");
						if (!IsAvailable(ptrEventNotification) || !IsReadable(ptrEventNotification))
						{
							continue;
						}
						ptrEventNotification->SetIntValue(ptrEventNotificationOn->GetValue());
					}
				}
			}
			catch (Spinnaker::Exception &e)
			{
				ROS_FATAL("Failed to configure device event handler");
			}
		}
		~DeviceEventHandler()
		{
			// this is how the spinnaker examples release the camera pointer and allow the nodelet to exit cleanly
			m_cam_ptr = nullptr;
		}
		void OnDeviceEvent(gcstring eventName)
		{
			if(eventName == "EventExposureEnd")
			{
				// lock the mutex to prevent changes to member timestamp object
				timestamp_mutex.lock();
				// get the now time as the end of the exposure
				m_last_frame_time = ros::Time::now();
				// get exposure time from the camera (# uSec that shutter was open)
				double exposure_time = double(m_cam_ptr->ExposureTime.GetValue());
				// convert uSec to Sec
				exposure_time /= 1000000.0;
				ROS_INFO("GOT EXP TIME : %f", exposure_time);
				// get half the exposure time
				exposure_time /= 2.0;
				// subtract from the end of exposure time to get the middle of the exposure
				m_last_frame_time -= ros::Duration(exposure_time);
				// unlock the mutex 
				timestamp_mutex.unlock();
			}
			
		}
		ros::Time get_last_exposure_end()
		{
		
			if(m_last_frame_time.toSec() == 0.0)
			{
				return ros::Time(0,0);
			}
			else
			{
				// lock the mutex to prevent changes to the member timestamp object
				timestamp_mutex.lock();
				// get the last frame time
				ros::Time stamp = m_last_frame_time;
				// assign the member timestamp to 0 to indicate it's already been reported
				m_last_frame_time = ros::Time(0,0);
				// unlock the mutex 
				timestamp_mutex.unlock();
				// return the stamp
				return stamp;
			}
		}
	private:
		// member mutex to lock the timestamp member when is set/get
		std::mutex timestamp_mutex; 
		// initialize the timestamp member to 0.0 to indicate it has not been set
		ros::Time m_last_frame_time = ros::Time(0,0);
		// Camera pointer to spinnaker camera object (used to get the current exposure time)
		CameraPtr m_cam_ptr;
};
#endif // DEV_EVENT_HANDLER