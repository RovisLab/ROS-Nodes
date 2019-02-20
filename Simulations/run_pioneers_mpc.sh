#!/bin/bash

script_pid=$$

echo "### Process ID: $script_pid"

echo "## Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:=test_4 &
pid="$pid $!"
sleep 5s

echo "## Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch pioneer_model:=pioneer_kinect.urdf  pose_file:=pioneer_poses &
pid="$pid $!"
sleep 1s

echo "## Launching map server..."
roslaunch pioneer_nav2d map_server.launch map_name:=test_4 &
pid="$pid $!"
sleep 5s

for i in {1..2}; do
    echo "## Robot: 'pioneer$i'"
    echo "# Launching 'pioneer$i' in Gazebo stack..."
    roslaunch pioneer_description pioneer_description.launch robot_name:=pioneer$i pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" use_kinect:=true sim:=true real_kinect:=false &
    pid="$pid $!"
    sleep 5s

    echo "# Launching AMCL localisation stack for 'pioneer$i'..."
    roslaunch pioneer_nav2d localisation_launcher.launch robot_name:=pioneer$i x:="$(rosparam get /pioneer$i/x)" y:="$(rosparam get /pioneer$i/y)" yaw:="$(rosparam get /pioneer$i/a)" &
    pid="$pid $!"
    sleep 5s

    echo "# Launching move_base stack for 'pioneer$i'"
    roslaunch pioneer_nav2d single_navigation.launch robot_name:=pioneer$i x:="$(rosparam get /pioneer$i/x)" y:="$(rosparam get /pioneer$i/y)" yaw:="$(rosparam get /pioneer$i/a)" movement_type:=navigation controller:=mpc &
    pid="$pid $!"
    sleep 5s
done

echo "## Launching MultiAgent MPC..."
roslaunch pioneer_nav2d ma_diff_mpc.launch &
pid="$pid $!"
sleep 5s

echo "## Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:=MPC_usage_two_pioneers &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 8h # A good 8h sleep
