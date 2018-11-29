#Gazebo version (simulated environment)

Description:

  Robot simulation is an essential tool in every roboticist's toolbox.Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments.
  For our project we introduced our Pioneer mobile robots and simulated our tools to adjust and have a good feedback from them before we use them on the real ones.
  The main idea of the project is creating a software that can make a car to drive itself autonomously, in our case, a Pioneer3-DX mobile robot.
The mapping tool helps the robot to create a map of the environment that can be stored and used for navigation.
The navigation tool gives the robot the power and knowleadge to go by itself to a given point on the map, without any help from outside.
 
How to make them run:
a.  Mapping tool:
  The script that starts the whole simulated mapping tool is the following and can be launched by opening a terminal in the current folder and running it by typing:
  $ ./run_pioneer_mapping

b. Navigation tool:
  We have two scripts that can make the full usage of this tool.First, the one that start all the navigation tool, which is mandatory:
1.  $ ./run_pioneer_navigation
  The second one can be used in a multi pioneer scenario where we want to send all the pioneers near a 2D goal.
2.  $ ./goal_hardcoded 

USAGE:
a. Mapping tool:
  In order to create a map of an environment we need a robot equiped with a laser equipment.For this Gazebo can give us plugins for any types of laser data devices,but for our situation we implemented only 2 types.To equip the robot with one of the following device you can change the "pioneer_model" argument from the runfile:
- Hokuyo laser: multi_pioneer_hokuyo.urdf
- Kinect camera: multi_pioneer_kinect.urdf
  
  Now that we have a robot equiped with a laser device, we need to add an evironment.In Gazebo we created few environements that can be chosen by changing the "world" argument with one of the following:
- maze_one
- maze
- willow_office
- umt_demo8
- umt_0

  After this save the script and run it. This will start the following:
		-roscore 
		-gazebo that contains one Pioneer3-dx equiped with the chosen laser and a world.
		-navigation stack for the robot(uses SLAM Gmapping).
		-controller for the move_base of the robot.
		-RViz with visualization of: 
			-robot model
			-laserscan
			-SLAM map
			-global and local planned path
			- Goal and Planned path

	To move the robot in the environment in order to scan it use you can either:
  - use "2DNav Goal" 
  - open a terminal and run the following:
    $ roslaunch pioneer_control pioneer_teleop

  After you're done, if you want to save the map, run the following in a terminal, into wanted location(where the map will be saved):
		$ rosrun map_server map_saver -f <your_map_name>

b.  Navigation tool:






The goal can be set in RViz via the 2D NAV Goal button.

In the 5 robots case you can change to which robot the 2D NAV Goal button reffers by changing the topic name.The default topic is set as :
  pioneer5/move_base_simple/goal
where pioneer5 is the name of the robot.

NOTE:
If the runfile does not run, change the permision of running it from the proprieties tab.

The dependecies are given in this project under the following packages:
1.  pioneer_description
2.  diff_drive
3.  move_base
Important:
the navigation to the goal takes in consideration for each robot only it's odometry and controls the robot via the "cmd_vel" topic.NO LASER USED so the robots might knock each other or get stuck on the map, so play nice :P.



note:
          Gmapping is good with hokuyo and kinect.watchout cuz i slowed down the move_base velocity for the robot.see where.tomorrow test the nav tool with 5 robots.
          first spawn 5 robots in the new created map.
          get for everyone an rviz visualization.
          test navigation without dynamic colission
          check dynamic colision and what is the max speed.
