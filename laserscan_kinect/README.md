# laserscan_kinect package

This package transforms the 3d pointcloud data from kinect, to laserscan type and publishes it to the "/scan" topic.

  USAGE:
1.  Open the terminal and run :
  $ roslaunch laserscan_kinect laserscan1.launch
Note: It starts the data aquisition package called "freenect_launch", the data transformation earlier mentioned, the tf transformation to connect the camera to the real robot's base_link, and it runs an RViz live visualization.

For tree frame visualization look into the "frame_tree folder".

