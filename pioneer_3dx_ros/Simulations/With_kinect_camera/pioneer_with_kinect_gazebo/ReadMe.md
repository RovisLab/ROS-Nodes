#pioneer with kinect simulation

#make kinect do slam try nr. 2

	Operating system:
	-Ubuntu 14.04
	
	Prerequisites:
	-ROS Indigo ROS Indigo:http://wiki.ros.org/indigo/Installation/Ubuntu
	
  Usage:
1.  Run the script into the command line:
  $ ./run_pioneer_kinect_gazebo
	Note:
		if you cannot run the script, run:
	$ chmod +x run_pioneer_kinect_gazebo and then the command from 1.

           ####What is it created from:####

- pioneer description package,replaced the hokuyo laser description and plugins with the ones of the kinect camera.

        What does it does:
- runs gazebo with "closed maze" world and pioneer with kinect camera.
- runs <pointcloud_to_laserscan>that transforms the depth points into laserscan messages.
    -it uses this packages instead of <depthimage_to_laserscan> node hoping it works.NOPE.



        NOTE:
- The good part:kinect is linked as a frame with the base_link.watch the description file if you want to.

        ACTUAL STATE:
- robot model problem with it's caster wheel position.
- the kinect camera shows nothing onto it's topics.

