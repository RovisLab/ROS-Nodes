# pioneer_3dx_ros
Pioneer3dx Mobile Robot mapping and navigation

Installation of the packages:
	
1.  open a terminal into your workspace "src" folder after you cloned the project and get the prequisites:
  - simple-navigation-goal package:
      $ git clone https://github.com/JenJenChung/simple_navigation_goals.git
  - also need to get rosaria for real life development; We will integrate rosaria into our project after we optimize it.(alongside amr_ros_config)
      $ git clone https://github.com/amor-ros-pkg/rosaria.git

2.  go into your workspace folder in the terminal (cd ..) and run the following, to resolve the general dependecies:
  $ rosdep install --from-paths src --ignore-src --rosdistro=kinetic

3.  finally build the project using:
  $ catkin_make

Tip:if you want to source your workspace by default when opening a new terminal, run the following commands and then restart the terminal:
  $ echo "source ~/your_workspace/devel/setup.bash" >> ~/.bashrc
  $ source ~/.bashrc

            Available tools
For each tool follow the path given to reach to it's "Readme.md" for information about how to use it and details about the nodes used.

1. For the simulated environment:
  a.  Laser device used: Hokuyo laser.
- Mapping with a Pioneer 3-DX mobile robot.
    Path: "Simulations/With_hokuyo_laser/mapping"

- Multi Robot navigation on an already mapped environment.
    Path: "Simulations/With_hokuyo_laser/multi_robot_navigation"

  b.  Laser device used: Kinect camera
- Mapping with a Pioneer 3-DX mobile robot.
    Path: "Simulations/With_kinect_camera/pioneer_base_fusioned_with_turtlebot_mapping/mapping_with_kinect"

- Multi Robot navigation on an already mapped environment.  ---!!!! WORK IN PROGRESS !!!!---
    Path: "Simulations/With_kinect_camera/pioneer_base_fusioned_with_turtlebot_mapping/navigation_tool"

- Bonus: as a model, we tried to use the turtlebot mapping tool and navigation.
    Path: "Simulations/Turtlebot_example"

2.  For the real environment:
  WIP,  the mapping with kinect works, we just need to get the packages used on the real robot, create the "ReadMe.md" file and the runfiles to propper use the tool.
 To Be Continued...  



