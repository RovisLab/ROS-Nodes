#!/bin/bash
source ~/my_local_repo/devel/setup.bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching map_saver..."
rosrun map_server map_saver -f $1 &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
