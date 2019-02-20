#Gazebo version (simulated environment)

Description:

  Robot simulation is an essential tool in every roboticist's toolbox.Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments.
  For our project we introduced our Pioneer mobile robots and simulated our tools to adjust and have a good feedback from them to get a good configuration with the real ones.
  The main idea of the project is creating a software that can make a car to drive itself autonomously, in our case, a Pioneer3-DX mobile robot.
The mapping tool helps the robot to create a map of the environment that can be stored and used for navigation.
The navigation tool gives the robot the power and knowleadge to go by itself to a given point on the map, without any help from outside.
 
How to make them run:
A.  Mapping tool:
  The script that starts gazebo and the other nodes that register the map is the following and can be launched by opening a terminal in the current folder and running it by typing:
  $ ./run_pioneer_mapping.sh

  In order to teleoperate the robot you can use this command into a terminal:

  $ roslaunch pioneer_control pioneer_teleop.launch
  
  After you scanned all environment you can save the map by calling :
  
  $ rosrun map_server map_saver -f ~/<destination folder>/<your_map_name>
Note : It's recommended to save the map inside <pioneer_gazebo>/map folder.

B. Navigation tool:
  We have two scripts that can make the full usage of this tool.
1.  First, the one that start all nodes required to make the robots able to navigate on a known map, which is mandatory:
  
    $ ./run_pioneer_navigation.sh

2.  The second one can be used in a multi pioneer scenario where we want to send all the pioneers near a 2D goal.
  
    $ ./goal_hardcoded.sh 




Arguments used into the simulation tools:

~world (string, default: "maze_one")

    The world loaded into the gazebo environment.Loaded from <pioneer_gazebo>/worlds. 
    Some examples:
  - maze_one
  - maze
  - willow_office
  - umt_demo8
  - umt_0

~pioneer_model (string, default: "multi_pioneer_kinect.urdf")

    URDF model of the entire unit spawned. It contains the robot model equiped with the laser device.It's loaded from <pioneer_description/>urdf folder.
    Models available for gazebo:
  - multi_pioneer_kinect.urdf : Pioneer 3-DX model equiped with the Kinect camera.
  - multi_pioneer_hokuyo.urdf : Pioneer 3-DX model equiped with the Hokuyo laser.

~pose_file (string, default: "pioneer_poses")

    *.yaml file containing robots name with their x, y, yaw coordinates into the world.Origin of the file: <pioneer_description>/params/pioneer_poses.yaml.
    Example robot pose:
    "pioneer2:
      x: 1.0
      y: 0.7
      a: -1.57"

~robot_name (string, default: "pioneer$i")
  
    robot's namespace for nodes used inside the called tool. $i represents the iteration inside the for loop.


~pose (string, default: "x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)")

    Each robot initial pose used inside the nodes.

~use_kinect(bool, default: "false")
  
    If <true>,starts the <laserscan_kinect> node that converts the depth data from kinect into laserscan messages.

~sim(bool, default: "true")

    If you want to simulate the tool, set the argument to <true>
    If you want to use the real robots, set the argument to <false>

~real_kinect(bool, default: "false")
  
    If you use the tool on the real robot set the argument to <true>.It will start the data aquisition from the Kinect camera.<false> means that gazebo will start the plugin for data aquisition.

~movement_type(string, default: "slow")
  
    Type of movement of the robots.it can be either <slow>, which is good for the mapping tool, either <fast>, which is good for the navigation tool.

~controller(string, default: "dwa")
  
    The type of controller used by the navigation package to follow the global path.
    Configuration available:
  - dwa :Dynamic Window Approach:  Given a global plan to follow and a costmap, the local planner produces velocity commands to send to a mobile base.
  - mpc :Model Predictive Control: Advanced method of process control that is used to control a process while satisfying a set of constraints.

~move_base_type(string, default:move_base_pioneer)
	
		Option to use the normal move_base package, or the one where you give a ".txt" with the path
		Configuration available:
  - move_base_pioneer :Default <move_base> integration.
  - move_base_test : The ".txt" file must be placed into <move_base_test>/resources/ folder and have the name file "global_path_pioneerX.txt", where x is the number of the robot.

~rviz_config(string, default: "one_pioneer_mapping")

    Configuration file loaded into RViz.Located into <pioneer_description>/RViz folder.
    Confgurations available:
  - one_pioneer_mapping : One Pioneer 3-DX model, map to be discovered, Global and Local plan in case of using the 2D Nav Goal tool.
  - multi_pioneer_rviz : Five Pioneer 3-DX models, map loaded from <map_server> package,for each one: Global plan, Local plan and Local CostMap. 

~goal_file(string, default: "goal")
  
    Load goal params of the robots from a file.Located into <pioneer_to_goal>/param folder.

pioneer_number(int, default: 1)

    Represents the number of the robot included into the goal_hardcoded runfile.Used to calculate the final position of each spawned robot.Placed inside the for loop that starts from 1 to the number of the robots that have been spawn.

~x(float, default: 5)

    x goal coordinate for a robot.

~y(float, default:5.5)

    y goal coordinate for a robot.

