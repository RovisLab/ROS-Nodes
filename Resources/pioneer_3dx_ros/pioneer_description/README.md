# pioneer_description

  This package contains models of the used robots and data acquisition drivers.

  The launchfiles provide the initialization of robot's model into the ROS Params server, etc.
  
  <pioneer_initialization.launch>
    Load the URDF into the ROS Parameter Server and the robot initial position.

Args:
~pioneer_model(string, default: "pioneer_hokuyo.urdf")

    URDF model of the robot.
    Available models:
  - pioneer_hokuyo.urdf : Pioneer 3-DX equiped with a Hokuyo laser for gazebo.
  - pioneer_kinect.urdf : Pioneer 3-DX equiped with a Kinect camera for gazebo.
  - pioneer_kinect_real.urdf : Pioneer 3-DX equiped with a Kinect camera used with the real robot.
  - pioneer_real.launch : Pioneer 3-DX used for the real robot.

~pose_file (string, default: "pioneer_poses")

    *.yaml file containing robots name with their x, y, yaw coordinates into the world.Origin of the file: <pioneer_description>/params/pioneer_poses.yaml.
    Example robot pose:
    "pioneer2:
      x: 1.0
      y: 0.7
      a: -1.57"

Example usage:
   
  $ roslaunch pioneer_initializaiton.launch pioneer_model:=pioneer_real.launch pose_file:=pioneer_poses


  <pioneer_description.launch>
    Spawns the initialized robot into gazebo or connects to the real robot.If kinect drivers available starts the conversion node for data acquisition.

Args:

~robot_name (string, default: "pioneer1")
  
    robot's namespace for nodes used inside the called tool. <$i> represents the iteration inside the for loop.


~pose (string, default: "x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)")

    Each robot initial pose used inside the nodes.Loaded from ROS Parameters Server.

~use_kinect(bool, default: "false")
  
    If <true>,starts the <laserscan_kinect> node that converts the depth data from kinect into laserscan messages.

~sim(bool, default: "true")

    If you want to simulate the tool, set the argument to <true>
    If you want to use the real robots, set the argument to <false>

~real_kinect(bool, default: "false")
  
    If you use the tool on the real robot set the argument to <true>.It will start the data acquisition from the Kinect camera.<false> means that gazebo will start the plugin for data acquisition.

Example usage:

  $ roslaunch pioneer_description pioneer_description.launch robot_name:=pioneer$i pose:="-x $(rosparam get /pioneer1/x) -y $(rosparam get /pioneer1/y) -Y $(rosparam get /pioneer1/a)" use_kinect:=true sim:=true real_kinect:=false


    <pioneer_visualization.launch>
      Starts RViz visualization tool.

Args:

~rviz_config(string, default: "one_pioneer_mapping")

    Configuration file loaded into RViz.Located into <pioneer_description>/RViz folder.
    Conifgurations available:
  - one_pioneer_mapping : One Pioneer 3-DX model, map to be discovered, Global and Local plan in case of using the 2D Nav Goal tool.
  - multi_pioneer_rviz : Five Pioneer 3-DX models, map loaded from <map_server> package,for each one: Global plan, Local plan and Local CostMap. 

Example usage:
  
  $ roslaunch pioneer_visualization.launch rviz_config:=one_pioneer_mapping
