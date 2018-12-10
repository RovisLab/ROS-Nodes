#!/bin/bash
 
my_pid=$$
echo "My process ID is $my_pid"

echo "loading destination:"
roslaunch pioneer_to_goal goal_coords_loader.launch goal_file:=goal &
pid="$pid $!"

echo "Sending goal to the pioneers"
for i in `seq 1 3`;
do
  roslaunch pioneer_to_goal pioneer_goal_hardcored.launch pioneer_number:=$i x:="$(rosparam get poz_x)" y:="$(rosparam get poz_y)" &
  pid="$pid $!"
  sleep 5s
done

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h

