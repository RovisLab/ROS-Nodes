#!/bin/bash
 
my_pid=$$
echo "My process ID is $my_pid"

echo "Starting darknet_ros"
roslaunch darknet_ros darknet_ros.launch &
pid="$pid $!"

echo "Sending goal to the pioneers"
for i in `seq 1 2`;
do
  roslaunch pioneer_to_goal pioneer_goal_darknet.launch pioneer_number:=$i &
  pid="$pid $!"
  sleep 5s
done

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h

