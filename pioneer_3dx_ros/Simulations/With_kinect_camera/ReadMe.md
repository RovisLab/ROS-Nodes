#kinect simulation for mapping.

        The best results come from <pioneer_base_fusioned_with_turtlebot_mapping> folder, 
            ./run_pioneer_kinect_simulation_try runfile.
            
           
so far the goal is to make a runfile that start the mapping action with pioneer equiped with kinect camera.
so into that runfile I tried to make this work from turtlebot simulator:
< Mapping an environment by driving around it.(it's the run_turtlebot_kinect_simulation runfile i created.)

    Open 5 terminal tabs.
    In terminal 1:$roscore
    In terminal 2:$roslaunch turtlebot_gazebo turtlebot_world.launch
    In terminal 3:$roslaunch turtlebot_gazebo gmapping_demo.launch
    In terminal 4:$roslaunch turtlebot_rviz_launchers view_navigation.launch
    Follow instructions here to change cost map.
    In terminal 5:$roslaunch turtlebot_teleop keyboard_teleop.launch
Drive around the robot to build your map. Now to save the map.

    Create a directory $mkdir ~/turtlebot_custom_maps
    $rosrun map_server map_saver -f ~/turtlebot_custom_maps/tutorial
>

Next step,what we want is autonomous driving.My ideea is too look at these steps from that turtlebot simulator,look into these launchfiles, and modify our packages to use the following nodes:

 Autonomous driving from turtlebot simulator:

    In one terminal run:$roslaunch turtlebot_gazebo turtlebot_world.launch
    In another terminal run:$roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/USERNAME/turtlebot_custom_maps/tutorial.yaml
    In another terminal run:$roslaunch turtlebot_rviz_launchers view_navigation.launch
    Send a navigation goal to the robot by clicking on 2D Nav Goal button in rviz (in the top bar).
		
		that "amcl_demo" is our interest node.(it somehow connects the 2d nav goal button from rivz with the moving of the base.)
		
			I tried to use "move_base" package from pioneer simulation,but i had errors and warnins :(.
			
			Good luck, mates!

            
