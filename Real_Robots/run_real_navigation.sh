#!/bin/bash
 
my_pid=$$
echo "My process ID is $my_pid"

echo "Launching pioneer_description..."
roslaunch pioneer_description pioneer_initialization.launch robot_URDF_model:="pioneer_kinect_real" pose_file:="pioneer_poses" &
pid="$pid $!"
sleep 5s

echo "Launching map server..."
roslaunch pioneer_nav2d map_server.launch map_name:="institut3.1" use_sim_time:=false &
pid="$pid $!"
sleep 5s

echo "Launching RosAria stack..."
for i in `seq 1 1`;
do
  roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer$i" robot_pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" environment:="real_world" use_real_kinect:=true kinect_to_laserscan:=true robot_username:="$(rosparam get /pioneer$i-username)" connect_robot_pc:=true &
  pid="$pid $!"
  sleep 5s
done

echo "Launching AMCL localisation stack..."
sleep 5s
for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d localisation_launcher.launch robot_name:=pioneer$i x:="$(rosparam get /pioneer$i/x)" y:="$(rosparam get /pioneer$i/y)" yaw:="$(rosparam get /pioneer$i/a)" robot_username:="$(rosparam get /pioneer$i-username)" connect_robot_pc:=true &
  pid="$pid $!"
  sleep 10s
done

echo "Launching move_base stack..."
sleep 5s
for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d move_base.launch robot_name:="pioneer$i" move_base_type:="move_base" base_global_planner:="NavfnROS" base_local_planner:="TebLocalPlannerROS" mcp_use:=false robot_username:="$(rosparam get /pioneer$i-username)" connect_robot_pc:=true &
  pid="$pid $!"
  sleep 10s
done

echo "Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:="two_pioneers_navigation_teb" &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h

