#Real Robots control.

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
done, just have to complete the readme.

Navigation:
Done, just have to complete the readme.
