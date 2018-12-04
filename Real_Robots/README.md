#Real Robots control.

<<<<<<< Updated upstream
First step:
1. Make sure you cloned the repo to each robot and are logged on.
2. Install openssh on each robot and onto your laptop.

  $ sudo apt-get install openssh-server openssh-client

3. Make sure all robots and the laptop are connected to the same network.

4. Setup environmental variables for ROS:
- Laptop ROS variables: 
  Open a terminal and run:

  $ echo "export ROS_IP=<laptop_ip>" >> ~/.bashrc
  
  $ echo "export ROS_MASTER_URI=http://<laptop_ip>:11311" >> ~/.bashrc

- Real Robots variables:
  Open a terminal, or connect to the robot from the laptop via SSH and run:
   
  $ echo "export ROS_IP=<robot_ip>" >> ~/.bashrc
  
  $ echo "export ROS_MASTER_URI=http://<laptop_ip>:11311" >> ~/.bashrc

After executing these commands close all terminals.
  

Usage:

A.  Mapping:
1.  Start the ROS_MASTER and initialize the robot model onto the Laptop:
  
- From this folder open a terminal and run:

  $./run_master_mapping.sh

2. Connect to the robot's controller and start the mapping tools:

- This step must be performed inside the same folder as this, but from the real robot.First we have to connect to the robot via ssh:

  $ ssh <robot_name>@<robot_ip>

-Enter the robot's password and after that you are connected to the terminal of the real robot.Go to the same folder of the robot where this file exists.Eg.:

  $<robot_name>@<robot_ip> cd ~/catkin_ws/src/ROS-Nodes/Real_Robots

-Now just run the follwing command to start the script for mapping:

  $<robot_name>@<robot_ip> ./run_real_pioneer_mapping.sh

3.  In order to have a real-time visualization of the map and control the robot's movement open another terminal in the same location on the Laptop and run the visualization tool:

  $ ./run_visualization.sh

Note:make sure that the runfile has the <rviz_config> argument set to <one_pioneer_mapping>.

4.  From this, you have two option to move the robot into the real environment:

a. Using the <2D NAV Goal> from the toolbar of RViz.

b. By manually teleoperatin the robot movement from the keyboard.
-Open a terminal and launch the teleoperation node:

  $ roslaunch pioneer_control pioneer_teleop

Note:the direction keys are explained in the terminal.

5.  After you are pleased with the recreated map, you can save it into the <pioneer_gazebo package>/map folder for further use (navigation) by executing the following node into a new terminal:

  $ rosrun map_server map_saver -f <pioneer_gazebo path>/src/<map_name>

Where the <map_name> is the one given by yourself.


B.  Navigation
1.  Start the ROS_MASTER and initialize the robot and map model onto the Laptop:
  
- From this folder open a terminal and run:

  $./run_master_navigation.sh

- This will first load the robots model and initial position.
Note: In this runfile we sent to the Master the initial 2D position of the robots via a *.yaml file placed into <pioneer_description folder>/params

- Also, the map created in the mapping tool can be added to the navigation tool by editing this runfile and replace the <map_name> argument to the name of the map.

2.  Positionate the robots as close to the position mentioned in the file earlier mentioned.

3. Connect to each robot's controller and start the navigation tools.
    Repeat this for each real Robot!

- This step must be performed inside the same folder as this, but from the real robot.First we have to connect to the robot via ssh:

  $ ssh <robot_name>@<robot_ip>

-Enter the robot's password and after that you are connected to the terminal of the real robot.Go to the same folder of the robot where this file exists.Eg.:

  $<robot_name>@<robot_ip> cd ~/catkin_ws/src/ROS-Nodes/Real_Robots

-Now just run the follwing command to start the script for mapping:

  $<robot_name>@<robot_ip> ./run_real_pioneer_navigation.sh

3.  In order to have a real-time visualization of the robots position and movement on the known map, open another terminal in the same location on the Laptop and run the visualization tool:

  $ ./run_visualization.sh

Note:make sure that the runfile has the <rviz_config> argument set to <multi_pioneer>.

4.  From this, you have two option to move the robot into the real environment:

a. Using the <2D NAV Goal> from the toolbar of RViz.
- This will make the first robot navigate to a given point. If you want to make the others navigate, change the	<2D NAV Goal topic> to <pioneer<number>/move_base_simple/goal>.

b. Using the	<goal_darket> runfile.
- This will need data from a real robot.It used the depth cloud data and darknet's person recognition node to get the 3d coordinates of a detected person over the camera and send it to the robots.
Connect a Kinect camera to the laptop, place it in the center of the room( (0, 0) position of the map) and run the earlier mentioned runfile placed in the same folder as this Readme:

  $ ./goal_darknet.sh

Note:This is set to send only two robots to the detected person.You can edit the runfile to send as many robots you started to use navigation by modifying the max iteration on the for loop.
