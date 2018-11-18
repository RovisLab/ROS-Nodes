#Pioneer_to_goal

Usage:
If you want to start a simulation with only one pioneer run the following script:
  $ ./run_one_pioneer_and_goal

If you want to start a simulation with five pioneers run the followin script instead:
  $ ./run_five_pioneers_and_goal

  For both runfile the robot model can be choose from the followin models, changing the "pioneer_model" argument from the runfile:
- simple Pioneer model : multi_robot_base/pioneer3dx
- Pioneer equiped with an Hokuyo laser: multi_pioneer_hokuyo.urdf
- Pioneer equiped with a Kinect camera: multi_pioneer_kinect.urdf

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
