Navigation Stack Setup

This section describes how to setup and configure the navigation stack on a robot. It assumes that all the requirements above for robot setup have been satisfied. Specifically, this means that the robot must be publishing coordinate frame information using tf, receiving sensor_msgs/LaserScan or sensor_msgs/PointCloud messages from all sensors that are to be used with the navigation stack, and publishing odometry information using both tf and the nav_msgs/Odometry message while also taking in velocity commands to send to the base. If any of these requirements are not met on your robot, please see the Robot Setup section above for instructions on completing them. 

# WIP
