# laserscan_visualizer

    !!WIP!!

This packages is used to connect to the laserscan data from a robot and visualize it in a window, without using RViz.

## Usage

1. Realtime usage:
* Open a terminal and run:
    $ rosrun laserscan_visualizer listener

2.WIP

### Node: laserscan_visualizer

This node visualize a robot's laser scan data in a window.


#### Subscribed Topics

* **`/scan`** ([sensor_msgs/LaserScan])

    The input laser scan data information.

### Node: image_and_laserscan_visualizer

This node visualize a robot's laser scan and image data in separate windows.


#### Subscribed Topics

* **`/scan`** ([sensor_msgs/LaserScan])

    The input laser scan data information.

* **`/camera/depth/image_raw`** ([sensor_msgs/Image])

    The input image data information from the camera.
