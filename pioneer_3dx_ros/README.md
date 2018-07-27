# pioneer_3dx_ros
Pioneer3dx gazebo ros simulator

Usage:
	git clone into your workspace.
	before doing "catkin_make" resolve the dependencies using:
	$ rosdep install --from-paths src --ignore-src rosdistro=indigo

Prequisites:
	-gazebo plugins:
		-control: 
		$ sudo apt-get install ros-indigo-gazebo-ros-control
	-map-server package":
		$ sudo apt-get install ros-indigo-map-server
	-pioneer2d-nav package:
		$ git clone https://github.com/JenJenChung/pioneer_2dnav.git & catkin_make
	-nav bundle into your workspace:
		$ git clone https://github.com/JenJenChung/nav_bundle.git  & catkin_make
	-amcl package:
		$ sudo apt-get install ros-indigo-amcl
	-simple-navigation-goal package:
		$ git clone https://github.com/JenJenChung/simple_navigation_goals.git & catkin_make
	Note: if there are any errors after you've got all the dependecies listed above, use again the folowing :
	$ rosdep install --from-paths src --ignore-src rosdistro=indigo
	

Project status:
	mapping tools with hokuyo laser available!

