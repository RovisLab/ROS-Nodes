# laserscan_kinect package

This package transforms the 3d pointcloud data from kinect, to laserscan type and publishes it to the "/scan" topic.

## Basic Usage

1.  Open the terminal and run :
  $ roslaunch laserscan_kinect laserscan_real.launch standalone:=false

Note: It starts the data aquisition package called "freenect_launch", the data transformation earlier mentioned, the tf transformation to connect the camera to the real robot's base_link, and it runs an RViz live visualization.

If you want to use it standalone set the argument <standalone> to true.

For tree frame visualization look into the "frame_tree folder".

### Node: laserscan_kinect

This node convert depth image to laser scan

### ROS related parameters

You can configure the camera data information inside `laserscan_kinect/config/params.yaml`.Each parameter has a description about itself.

#### Subscribed Topics

* **`/camera/depth/image_raw`** ([sensor_msgs/Image])

    The depth camera data information.
* **`/camera/depth/camera_info`** ([sensor_msgs/CameraInfo])

    Depth camera calibration and metadata.

#### Published Topics

* **`/scan`** ([sensor_msgs/LaserScan])

    The output laser scan.



