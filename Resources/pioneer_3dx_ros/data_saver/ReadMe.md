# data_saver

    This package connects to the robot's "laserscan", "image_raw", and "odom" topics, synchronize the messages by the timestamp of the "laserscan" topic, saves the laserscan and image topic as "*.png" format and information about odometry into a "*.txt" file along with the image name.

## Usage

1. /run_data_saver

2.WIP

### Node: state_server

This node creates a directory into this package in order to save the data onto a common location.it connects to a namespace of topics via the "robot_namepsace" argument.


#### Parameters

* **`robot_namespace`** 

    Name of the robot_namespace in the case of multi robot scenario.
#### Subscribed Topics

* **`/scan`** ([sensor_msgs/LaserScan])

    The input laser scan data information.

* **`/camera/rgb/image_raw`** ([sensor_msgs/Image])

    The input image_raw data from the camera.

* **`/odom`** ([nav_msgs/Odometry])

    The input position and velocity of the robot.


#### Args for the runfile
~robot_name (string, default: "pioneer$i")
  
    robot's namespace for nodes used inside the called tool. <$i> represents the iteration inside the for loop.

~use_bag (bool, default: true)
  
    Option of using a bag file instead of running the robots.Works only if you have only one robot in the bagfile.If you have a bagfile with multiple robots set this argument to false and play the bagfile separately before running this runfile.

bag_path(string, default: 2018-12-05-13-45-45.bag)

    In case you choose a bag_file, you have to place it inside the <data_saver> package and give the bag name via this argument.




