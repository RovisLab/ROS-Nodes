#!/bin/bash
source ~/my_local_repo/devel/setup.bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:=maze_one &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
