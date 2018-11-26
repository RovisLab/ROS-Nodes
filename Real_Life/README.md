#Real Robots control.

<<<<<<< Updated upstream
First step:
1. Make sure you cloned the repo to each robot and are logged on.
2. Install openssh on each robot and onto your laptop.
3. Make sure all robots and the laptop are connected to the same network.
4. Set the ROS_MASTER_URI of all robots as the IP of the laptop:
echo "export ROS_MASTER_URI=http://<laptop_ip>:11311" >> ~/.bashrc
5. Set to each robot ROS_IP
echo "export ROS_IP=<robot_ip>" >> ~/.bashrc


Usage:

a.  Mapping:
1.  
=======
##req
openssh server and client
ROS_MASTER_URI
ROS_IP

  $ ssh <robot_name>@<robot_ip>

 
Mapping:

  This folder:
  $ ./run_master_mapping

  New terminal:
   
  $ ssh <robot_name>@<robot_ip>
  go to same folder location and open a terminal, then run :
  $ ./run_real_pioneer_mapping

  Open another terminal in the same location of the Laptop and run the visualization tool:

  $ ./run_visualization

  In this runfile you can change the rviz_config argument to:   
- one_pioneer_mapping
- multi_pioneer


>>>>>>> Stashed changes
done, just have to complete the readme.

Navigation:

In this folder:
  $ ./run_master_navigation

  New terminal, for each robot:
  $ ssh <robot_name>@<robot_ip>
go to same folder location and modify the number for the robot:
  $ gedit run_real_pioneer_navigation

then open a terminal and run :
  $ ./run_real_pioneer_navigation
  
Done, just have to complete the readme.
