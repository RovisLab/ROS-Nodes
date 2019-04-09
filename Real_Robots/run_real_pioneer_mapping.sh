#!/bin/bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching Pioneers in Gazebo stack..."
for i in `seq 1 1`;
do
  roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer$i" robot_pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" environment:="real_life" use_real_kinect:=true kinect_to_laserscan:=true &
  pid="$pid $!"
  sleep 5s
done

echo "Launching SLAM Gmapping stack..."
sleep 5s
for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d gmapping_launcher.launch robot_name:="pioneer$i" &
  pid="$pid $!"
  sleep 10s
done

echo "Launching move_base stack..."
sleep 5s
for i in `seq 1 1`;
do
  roslaunch pioneer_nav2d move_base.launch robot_name:="pioneer$i" move_base_type:="move_base" base_global_planner:="NavfnROS" base_local_planner:="TebLocalPlannerROS" mcp_use:=false &
  pid="$pid $!"
  sleep 10s
done

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
