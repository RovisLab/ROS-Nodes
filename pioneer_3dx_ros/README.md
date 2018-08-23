# pioneer_3dx_ros
Pioneer3dx gazebo ros simulator

Usage:
	
  - git clone into your workspace.
  - get the prequisites;
  - before doing "catkin_make" resolve the dependencies using:
	$ rosdep install --from-paths src --ignore-src rosdistro=indigo

Prequisites:
	-simple-navigation-goal package:
		$ git clone https://github.com/JenJenChung/simple_navigation_goals.git & catkin_make
	-also need to get rosaria for pioneer2d_nav pkg:(hmmmmm)
		$ git clone https://github.com/amor-ros-pkg/rosaria.git & catkin_make

	Note: if there are any errors after you've got all the dependecies listed above, use again the folowing :
	$ rosdep install --from-paths src --ignore-src rosdistro=indigo
	

Project status:
	mapping tools with hokuyo laser available!

