#Multi_robot_navigation tool with kinect camera for virtual environment 

  Operating system:
- Ubuntu 16.04
	
  Prerequisites:
- ROS Kinetic ROS Kinetic:http://wiki.ros.org/kinetic/Installation/Ubuntu

	Usage:
1.  Mapping with hokuyo laser. 
  $ ./test_one_pioneer_one
	Note:
		if you cannot run the script, run:
	$ chmod +x test_one_pioneer_one and then the command from 1.	

*This will start the following:
		- roscore (no need to run it in another terminal)
		- gazebo that contains three Pioneer 3-DX, each equiped with a kinect laser and a world already created and with a given start position in the world.
    - map server that loads the already mapped world.
		- navigation stack for each robot(uses AMCL).
		- controller for the "move_base" package that guide the robot's movement.
		-RViz with visualization of: 
      - scanned map
      - 2 robot models each with:
        - laserscans
        - global and local planned path

2.  
a.  If you want to make each robot go to a preffered position, right click on the "2D NAV TOOl" and change it's called topic for the expected robot topic:
  "pioneer$i/move_base/simple" , where $i is the number of the robot.(the counting start at one)
b.  If you want to make all the robots go to a specific point, use the destination tool by running the following script:
  $ ./goal_sender_hardcoded.
Note:
		if you cannot run the script, run:
	$ chmod +x goal_sender_hardcoded and then the command before.

