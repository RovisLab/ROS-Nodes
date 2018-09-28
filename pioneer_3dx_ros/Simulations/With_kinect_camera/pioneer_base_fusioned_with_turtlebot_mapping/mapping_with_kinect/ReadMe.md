#pioneer with kinect simulation

  Operating system:
- Ubuntu 16.04
	
  Prerequisites:
- ROS Kinetic ROS Kinetic:http://wiki.ros.org/kinetic/Installation/Ubuntu

	Usage:
1. 
  $ ./run_pioneer_kinect_laserscan_gmapping

	Note:
		if you cannot run the script, run:
	$ chmod +x <name_of_the_runfile> and then the command from 1.

*This will start the following:
    - roscore
    - gazebo world with a Pioneer 3-DX equiped with a Kinect camera.
    - laserscan_kinect tool that transforms the depth data into laserscan in real time.
    - gmapping: the mapping tool.
    - move_base package: control over the movement of the robot.
    - controller for the movement of the robot, in case it gets stucked.
    - Rviz visualization tool:
      - robot model
      - laserscan
      - the map that is being created
2.	To move the robot in the environment use "2DNav Goal" in order to scan all the map. or run the following command in terminal to control the robot directly:
  $ roslaunch pionneer_control pioneer_teleop.launch
if you wanna use it with the destination tool use the folowing scripit:
    $  ./run_destination_tool

3.	After you're done, if you want to save the map, run the following in terminal, into wanted location(where the map will be saved):
		$ rosrun map_server map_saver -f <your_map_name>
    
 

