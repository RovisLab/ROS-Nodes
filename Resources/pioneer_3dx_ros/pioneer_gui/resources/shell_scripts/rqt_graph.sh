#!/bin/bash
my_pid=$$
 
echo "My process ID is $my_pid"

echo "Launching rqt_graph..."
rosrun rqt_graph rqt_graph &
pid="$pid $!"
sleep 5s

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
