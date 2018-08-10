#Mapping tool for virtual environment.	
  
  Operating system:
- Ubuntu 14.04
	
  Prerequisites:
- ROS Indigo ROS Indigo:http://wiki.ros.org/indigo/Installation/Ubuntu

	Usage:
1.  Mapping with hokuyo laser.  --done--
  $ ./run_pioneer_hokuyo_mapping
	Note:
		if you cannot run the script, run:
	$ chmod +x run_pioneer_mapping and then the command from 1.	
	
*This will start the following:
		-roscore (no need to run it in another terminal)
		-gazebo that contains one Pioneer3-dx equiped with hokuyo laser and a world already created.
		-navigation stack for the robot(uses SLAM Gmapping).
		-controller for the move_base of the robot.
		-RViz with visualization of: 
			-robot model
			-laserscan
			-SLAM map
			-global and local planned path
			- Goal and Planned path

2.	To move the robot in the environment use "2DNav Goal" in order to scan all the map.

3.	After you're done,if you want to save the map,run the following in terminal,into wanted location(where the map will be saved):
		$ rosrun map_server map_saver -f <your_map_name>

YEAH, this works excelent.
