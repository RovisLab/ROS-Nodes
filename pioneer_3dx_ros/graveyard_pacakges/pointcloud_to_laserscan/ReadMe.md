#pointcloud_to_laserscan package

Usage:
1.  a.If you have freenect packages installed use the follwoing command:
        $ roslaunch pointcloud_to_laserscan sample_node_freenect.launch

    b.If you have openni packages installed use the follwoing command:
        $ roslaunch pointcloud_to_laserscan sample_node_openni.launch

2.  run RViz setting the fixed frame to "camera_link" and add the laserscan topic.
        $ rosrun rviz rviz
    TODO:add to the launchfile a prepared RViz visualization.

How does it work:
  -it runs the data aquisition from the real kinect and transform the depth points topic into a fake laser_scan topic.


UPDATE:
  - This package is obsolete, used only for the simulation. if you want to obtain laserscan from kinect use the "laserscan_kinect" package.

