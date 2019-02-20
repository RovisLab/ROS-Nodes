#!/bin/bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:=test_4 &
pid="$pid $!"
sleep 5s

echo "Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch pioneer_model:=pioneer_kinect.urdf  pose_file:=pioneer_poses &
pid="$pid $!"
sleep 2s

echo "Launching Pioneers in Gazebo stack..."
for i in `seq 1 2`;
do
  roslaunch pioneer_description pioneer_description.launch robot_name:=pioneer$i pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" use_kinect:=true sim:=true real_kinect:=false &
  pid="$pid $!"
  sleep 5s
done

echo "Launching map server..."
roslaunch pioneer_nav2d map_server.launch map_name:=test_4 &
pid="$pid $!"
sleep 2s

echo "Launching AMCL localisation stack..."
for i in `seq 1 2`;
do
  roslaunch pioneer_nav2d localisation_launcher.launch robot_name:=pioneer$i x:="$(rosparam get /pioneer$i/x)" y:="$(rosparam get /pioneer$i/y)" yaw:="$(rosparam get /pioneer$i/a)" &
  pid="$pid $!"
  sleep 2s
done

echo "Launching move_base stack..."
for i in `seq 1 2`;
do
  roslaunch pioneer_nav2d single_navigation.launch robot_name:=pioneer$i x:="$(rosparam get /pioneer$i/x)" y:="$(rosparam get /pioneer$i/y)" yaw:="$(rosparam get /pioneer$i/a)" movement_type:=navigation controller:=dwa move_base_type:=move_base_pioneer &
  pid="$pid $!"
  sleep 2s
done

echo "Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:=DWA_vs_MPC &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
