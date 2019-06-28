#!/bin/bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:="posts_grouped" &
pid="$pid $!"
sleep 5s

echo "Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch robot_URDF_model:="pioneer_kinect" pose_file:="pioneer_poses" &
pid="$pid $!"
sleep 2s

echo "Launching Pioneers in Gazebo stack..."
for i in `seq 1 1`;
do
  roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer$i" robot_pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" environment:="gazebo" use_real_kinect:=false kinect_to_laserscan:=true &
  pid="$pid $!"
  sleep 5s
done

echo "Launching map server..."
roslaunch pioneer_nav2d map_server.launch map_name:="smap1" &
pid="$pid $!"
sleep 2s

echo "Launching AMCL localisation stack..."
for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d localisation_launcher.launch robot_name:="pioneer$i" robot_pose_x:="$(rosparam get /pioneer$i/x)" robot_pose_y:="$(rosparam get /pioneer$i/y)" robot_pose_yaw:="$(rosparam get /pioneer$i/a)" localization_type:="amcl" &
  pid="$pid $!"
  sleep 2s
done

echo "Launching move_base stack..."
for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d move_base.launch robot_name:="pioneer$i" move_base_type:="move_base" base_global_planner:="NavfnROS" base_local_planner:="TebLocalPlannerROS" mcp_use:=false &
  pid="$pid $!"
  sleep 2s
done

echo "Starting data_saver_node"
rosrun data_saver odom_path &
pid="$pid $!"
sleep 5s

echo "Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:="institut" &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
