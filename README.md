# ROS-Nodes

This repo stores all nodes developed by the RovisLab team. 
Also, all additional nodes availablele can be included into "Aditional Nodes" file. A shord description is also mandatory. 

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS Kinetic version]), which needs to be [installed](http://wiki.ros.org) first( download the ros-kinetic-desktop-full) 

In order to install our tools,first clone the latest version from this repository into your catkin workspace:

	$ cd ~/<your_repository>/src
	$ git clone https://github.com/RovisLab/ROS-Nodes


To make sure you have all dependecies required run the followin commands in the earlier opened terminal:
    	
	$ cd ..
	$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic


### Building

To maximize performance, make sure to build in *Release* mode. You can specify the build type by setting"

    $ catkin_make -DCMAKE_BUILD_TYPE=Release

Otherwise, you can just build it normally using the command:
	
	$ catkin_make

If there is an error about not finding the "rosaria"package go to your workspace src, open a terminal and run:

  $ git clone https://github.com/amor-ros-pkg/rosaria.git
Then run "catkin_make" again in the workspace.

### Tools Available

There are 2 main tools created in this project:

- Mapping tool
- Navigation tool

Each tool is meant to be workin into a simulated environment and in a real life usage.
For both situation we created folders that contain the scripts to be run and 'ReadMe.md' files that contain description of the used nodes and how to use the tools.


