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

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h

