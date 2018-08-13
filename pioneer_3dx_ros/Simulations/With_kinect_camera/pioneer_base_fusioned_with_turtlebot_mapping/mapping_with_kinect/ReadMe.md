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

	BRO! using 
		$ rosrun tf tf_echo map swivel 
	Shows this error
Failure at 1880.180000000
Exception thrown:"swivel" passed to lookupTransform argument source_frame does not exist. 
The current list of frames is:
Frame base_link exists with parent odom.
Frame odom exists with parent map.
Frame right_wheel exists with parent base_link.
Frame left_wheel exists with parent base_link.
Frame back_sonar exists with parent base_link.
Frame front_sonar exists with parent base_link.
Frame top_plate exists with parent base_link.
Frame camera_depth_frame exists with parent camera_rgb_frame.
Frame camera_rgb_frame exists with parent base_link.
Frame camera_depth_optical_frame exists with parent camera_depth_frame.
Frame camera_link exists with parent camera_rgb_frame.
Frame camera_rgb_optical_frame exists with parent camera_rgb_frame.
Frame center_hubcap exists with parent center_wheel.
Frame chassis exists with parent base_link.
Frame left_hub exists with parent left_wheel.
Frame right_hub exists with parent right_wheel.

		so the center_hubcap has the "center_wheel" as parent,but center_center wheel is an orhphane :'( ! =>fixed. the i had to add fake joint controller for the continuous joint of the caster wheel.

    next problem:
[ WARN] [1533738772.302324739, 1527.954000000]: The scan observation buffer has not been updated for 35.70 seconds, and it should be updated every 1.00 seconds.
[ WARN] [1533738772.312861045, 1527.962000000]: [/move_base]:Sensor data is out of date, we're not going to allow commanding of the base for safety
 that's what is happening when i try to add the move_base package that gives us the chance to send te robot to some coordinates via the "2D Nav Goal". It might happen because the sensor messages have a low frequency regarding to the camera we use for the "scan" topic. TRY TELEOP?
      =>FIXED : the params from the costmap were not correct.

      the cmd_vel topic is send by the move_base node, but we have no subscriber:
gatu270124@ER01556P:~$ rostopic info /cmd_vel 
Type: geometry_msgs/Twist

Publishers: 
 * /move_base (http://ER01556P:34075/)

Subscribers: None             => look into the turtlebot example to see the connection.or in the pioneer_with_hokuyo mapping example .
    =>partialy fixed: /pioner/cmd_vel was the right topic. i can start this roslaunch in another terminal only, because it does not allow me to run it from the runfile.

I added path planned for the 2dnavgoal.it sets the goal and the global and local path,but the robot does not move. i have to connect the move_base package to the cmd_vel of the robot.SOL?look at the mapping with hokuyo, or autonomous driving with turtlebot.
    ->fixed: took a launchfile from mybot_ws package from git that has a launchfile called mybot_teleop.i remapped the cmd_vel topic to our robot.For 2dnav goal i added the pioneer_ros launchfile that sends commands for the pioneer to go to the specified path . still cannot use the teleop in the same script,tho :(



the mapping is pretty slow, but the proof of concept is created.for navigation_only simulation script we'll use the map created with the hokuyo laser.
    
 

