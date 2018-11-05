#turtlebot_simulation

Prerequisites:

1.  $sudo apt-get update
2.  $sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-rocon-remocon ros-kinetic-rocon-qt-library ros-kinetic-ar-track-alvar-msgs

NOTE: If you build the workspace with the instructions given in the first Readme ignore these 2 steps.



Usage:
1. $./run_turtlebot_kinect_simulation.

2. open another terminal and run the following:
  $ roslaunch turtlebot_teleop keyboard_teleop.launch

  What is it:
- a kobuki robot base with a kinect camera and a stand for it.
- it reproduces the occupancy grid of a simulated world.
- it can only be teleoperated.

Note: This is the original mapping with a kobuki robot equiped with a kinect camera.
  
