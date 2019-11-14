# blackfly_nodelet

Simple ROS driver wrapping the spinnaker API for Blackfly Cameras.

## Timestamping
Images are timestamped using the End of Exposure event given by the Spinnaker API. When this event occurs, the current ROS time is saved in the device event handler class. The device event handler then queries the camera for its current exposure time. The exposure time is divided by 2, and this time is subtracted from the saved time stamp. This procedure is performed in order to move the image's timestamp to the middle of the camera's exposure. 

## Disclosure
This driver is untested and not field proven. Use at your own risk.

## Notes:
1. Camera Frame rate may drop if the camera is not connected to a USB3.0 port. 
2. If using auto exposure, auto gain, and gamma correction, the camera tends to choose the maximum exposure value.
3. Double Check:

In /etc/default/grub, change 
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"  
```
to 
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=32768"  
```
then 
```
sudo update-grub  
```
then restart and verify that 
```
cat /sys/module/usbcore/parameters/usbfs_memory_mb  
```
is 32768 or something like that
