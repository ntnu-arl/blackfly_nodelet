#ifndef DEV_EVENT_HANDLER_
#define DEV_EVENT_HANDLER_
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream> 
#include <ros/ros.h>
#include <deque>
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
class DeviceEventHandler : public DeviceEvent
{
	public:
		// This constructor registers an event name to be used on device events.
		DeviceEventHandler(CameraPtr cam_ptr)
		{
			INodeMap &node_map = cam_ptr->GetNodeMap();
			try
			{
				ROS_INFO("ENABLING Exposure Event");
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
						ROS_INFO("EXPOSURE END EVENT ENABLED!");
					}
				}
			}
			catch (Spinnaker::Exception &e)
			{
				ROS_ERROR("Failed to configure device event handler");
			}
			cam_ptr = nullptr;
		}
		~DeviceEventHandler()
		{

		}
		void OnDeviceEvent(gcstring eventName)
		{
			if(eventName == "EventExposureEnd")
			{
				m_last_exp_end = ros::Time::now();
				// ros::Time stamp
				// m_exposure_end_queue.push_back(stamp);
				// ROS_INFO("EXP END EVENT : %f", stamp.toSec());
			}
		}
		ros::Time get_last_exposure_end()
		{
		
			if(m_last_exp_end.toSec() == 0.0)
			{
				return ros::Time(0,0);
			}
			else
			{
				ros::Time stamp = m_last_exp_end;
				m_last_exp_end = ros::Time(0,0);
				return stamp;
			}
		}
	private:
		ros::Time m_last_exp_end = ros::Time(0,0);

};
#endif // DEV_EVENT_HANDLER