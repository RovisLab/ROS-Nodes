#pioneer with kinect simulation

#make kinect do slam try nr. 4

	Operating system:
	-Ubuntu 14.04
	
	Prerequisites:
	-ROS Indigo ROS Indigo:http://wiki.ros.org/indigo/Installation/Ubuntu

	Usage:
1.  A.    Mapping with kinect camera *ripped_off_from_turtlebot_mapping* --in progress--
  $ ./run_pioneer_kinect_simulation

    B.    Mapping with kinect camera *ripped_off_from_turtlebot_mapping* --in progress-- --take this--
  $ ./run_pioneer_kinect_simulation_try

	Note:
		if you cannot run the script, run:
	$ chmod +x <name_of_the_runfile> and then the command from 1.

<Actual state for A>:
  The little brother of D,has no rviz launchfile,it has problem with the /base_scan and /scan topics.go for D if you want to test it!

<ACTUAL STATE FOR B>:
  This runfile is created from turtlebot mapping pack.
I took the description files for kinect camera and fusioned it with the pioneer robot base.
- for mapping i used gmapping taken from turtlebot package.
- for fake laserscan i used pointcloud_to_laserscan package(sample_node.launch).
- i created an RViz launchfile with the information that we need.
 
  <INFO>:  It does not have any teleoperation for now,neither the move_base package needed in order to command it to walk alone to a given point(either from a node that sends the coords, either 2dnav_goal tool from rviz).
-  The problem is that move_base node watches for the laserscan topic,and it says that is not being updated...(i check it in rviz and says either that is not being published,or NAN).Maybe laserscan topic is too slow?check the move_base launcher.
  
  <HINT>:in robot model "swivel and caster wheel appear as not being joined with the whole robot(RViz=frame is not seen; Gazebo= caster wheel has no color?..TODO:see the materials for that, I guess)


    
 

