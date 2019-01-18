#!/bin/bash

script_pid=$$

echo "### Process ID: $script_pid"

echo "## Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:=maze_one &
pid="$pid $!"
sleep 5s

echo "## Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch pioneer_model:=pioneer_kinect.urdf  pose_file:=pioneer_poses &
pid="$pid $!"
sleep 5s

echo "## Launching map server..."
roslaunch pioneer_nav2d map_server.launch map_name:=maze_one &
pid="$pid $!"
sleep 5s

for i in {1..1}; do
    echo "## Robot: 'pioneer$i'"
    echo "# Launching 'pioneer$i' in Gazebo stack..."
    roslaunch pioneer_description pioneer_description.launch robot_name:=pioneer$i pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" use_kinect:=true sim:=true real_kinect:=false &
    pid="$pid $!"
    sleep 5s

    echo "# Launching AMCL localisation stack for 'pioneer$i'..."
    roslaunch pioneer_nav2d localisation_launcher.launch robot_name:=pioneer$i x:="$(rosparam get /pioneer$i/x)" y:="$(rosparam get /pioneer$i/y)" yaw:="$(rosparam get /pioneer$i/a)" &
    pid="$pid $!"
    sleep 10s

    echo "# Launcing move_base stack for 'pioneer$i'"
    roslaunch pioneer_nav2d single_navigation.launch robot_name:=pioneer$i x:="$(rosparam get /pioneer$i/x)" y:="$(rosparam get /pioneer$i/y)" yaw:="$(rosparam get /pioneer$i/a)" movement_type:=navigation controller:=dwa &
    pid="$pid $!"
    sleep 10s
done

echo "## Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:=one_pioneer_nav &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 8h # A good 8h sleep
