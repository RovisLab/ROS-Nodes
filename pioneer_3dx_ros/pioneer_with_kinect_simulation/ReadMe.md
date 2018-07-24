#pioneer with kinect simulation

	Operating system:
	-Ubuntu 14.04
	
	Prerequisites:
	-ROS Indigo ROS Indigo:http://wiki.ros.org/indigo/Installation/Ubuntu
	
  Usage:
1.  Run the script into the command line:
  $ ./run_pioneer_kinect_sim
	Note:
		if you cannot run the script, run:
	$ chmod +x run_pioneer_kinect_sim and then the command from 1.

  What does it does:
-runs gazebo with "closed maze" world and pioneer with kinect camera.
-runs rviz.so far empty.

  TODO: make a roslaunch to visualize interest topics from the simulation:
          -robot model
          -kinect topics.
          
