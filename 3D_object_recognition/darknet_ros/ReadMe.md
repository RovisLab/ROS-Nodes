# 3D Object detection with YOLO

The YOLO packages have been tested under ROS Kinetic and Ubuntu 16.04.

## How to run it:

If you want to run fast but with not so good detection use:

	$roslaunch darknet_ros darknet_ros.launch
	
Performance but slower in FPS 

	$roslaunch darknet_ros yolo_v3.launch
	
Debug mode

	$roslaunch darknet_ros darknet_ros_gdb.launch
	
	
