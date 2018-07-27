#mapping tool for simulation with kinect

#make kinect do slam try nr. 3

	Operating system:
	-Ubuntu 14.04
	
	Prerequisites:
	-ROS Indigo ROS Indigo:http://wiki.ros.org/indigo/Installation/Ubuntu	

	Usage:
1.  Mapping with hokuyo laser.  --done--
      $ ./run_pioneer_kinect_gazebo
	Note:
		if you cannot run the script, run:
	$ chmod +x run_pioneer_kinect_gazebo and then the command from 1.
    
	This will start the following:
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

  <ACTUAL STATE>:
laser vision is zero.
map not showing.all of this may be because some topics must be remapped.
when trying to use "2DNAV goal" it shows the following error in terminal:
<<[ERROR] [1532431869.243573231, 1472.646000000]: Robot semantic description not found. Did you forget to define or remap '/robot_description_semantic'?>>
