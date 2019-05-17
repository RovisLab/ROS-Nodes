#!/bin/bash
source ~/my_local_repo/devel/setup.bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:="$1" &
pid="$pid $!"
sleep 5s

echo "Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch robot_URDF_model:="$2" pose_file:="pioneer_poses" &
pid="$pid $!"
sleep 2s

echo "Launching Pioneers in Gazebo stack..."
for i in `seq 1 1`;
do
  roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer$i" robot_pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" environment:="$3" use_real_kinect:=$4 kinect_to_laserscan:=$5 &
  pid="$pid $!"
  sleep 2s
done

echo "Launching SLAM Gmapping stack..."
for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d gmapping_launcher.launch robot_name:="pioneer$i" gmapping_config_type:="$6" &
  pid="$pid $!"
  sleep 2s
done

echo "Launching move_base stack..."

for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d move_base.launch robot_name:="pioneer$i" move_base_type:="move_base" base_global_planner:="$7" base_local_planner:="$8" mcp_use:=$9 &
  pid="$pid $!"
  sleep 2s
done

echo "Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:="${10}" &
pid="$pid $!"

echo "$pid" > /home/$(whoami)/script_pid.txt

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
