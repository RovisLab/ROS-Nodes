# 3D Object detection with YOLO

The YOLO packages have been tested under ROS Kinetic and Ubuntu 16.04 with Xbox Kinect camera.

## Description

![Screenshot of detection](doc/Screenshot.png)

This Object deteciton node is optimised to detect only people. With the help of the kinect camera that comes with some depth information, it shows the coordinates of the person placement.

## Best usage

	roscore
	roslaunch freenect_launch freenect.launch
	roslaunch darknet_ros darknet_ros.launch

## How to get the 3D coordinates

![How i get the 3D coordinates](doc/img4.png)

	x_world = (x_screen - c_x) * z_world / f_x;
	y_world = (y_screen - c_y) * z_world / f_y;
	
The z_world coordinate is the depth provided by kinect camera, and c_x, c_y, f_x, f_y are the parameters provided by kinect when you calibrate it. If you don't want to calibrate the camera you could use the following parameters [from this link](https://github.com/OpenKinect/libfreenect/blob/master/examples/glpclview.c).

In this case,x_screen and y_screen is the middle point of the detected object in the bounding box.
You could get x_screen and y_screen with:

	x_screen=((xmax-xmin)/2)+xmin;
	y_screen=((ymax-ymin)/2)+ymin;
	
## How to get better FPS:

In yolo_network_config/cfg/yolov2-tiny.cfg change the following properties to lower numbers like:

    width=288
    height=288

## How to get more detection objects and improve the threshold

In config/yolov2-tiny.yaml you can add more objects from [this list](https://github.com/leggedrobotics/darknet_ros/blob/master/darknet_ros/config/yolov2-tiny.yaml) right under "names" parameter.
In config/yolov2-tiny.yaml you can decrease or raise the threshold. Put a number between 0.1 (10%) and 0.99 (99%) next to "value" parameter. 
	
	
## How to run it:

If you want to run fast but with not so good detection use:

	$roslaunch darknet_ros darknet_ros.launch
	
Performance but slower in FPS 

	$roslaunch darknet_ros yolo_v3.launch
	
Debug mode

	$roslaunch darknet_ros darknet_ros_gdb.launch


	


