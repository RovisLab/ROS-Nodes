#!/bin/bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching pioneer_key_teleop..."
rosrun pioneer_key_teleop pioneer_key_teleop_node &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
