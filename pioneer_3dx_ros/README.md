# pioneer_3dx_ros
Pioneer3dx Mobile Robot mapping and navigation

Usage:
	
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


