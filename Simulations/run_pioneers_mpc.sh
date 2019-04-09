#!/bin/bash

script_pid=$$

echo "### Process ID: $script_pid"

echo "## Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:="test_4" &
pid="$pid $!"
sleep 5s

echo "## Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch robot_URDF_model:="pioneer_kinect" pose_file:="pioneer_poses" &
pid="$pid $!"
sleep 1s

echo "## Launching map server..."
roslaunch pioneer_nav2d map_server.launch map_name:="test_4" &
pid="$pid $!"
sleep 5s

for i in {1..2}; do
    echo "## Robot: 'pioneer$i'"
    echo "# Launching 'pioneer$i' in Gazebo stack..."
    roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer$i" robot_pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" environment:="gazebo" use_real_kinect:=false kinect_to_laserscan:=true &
    pid="$pid $!"
    sleep 5s

    echo "# Launching AMCL localisation stack for 'pioneer$i'..."
    roslaunch pioneer_nav2d localisation_launcher.launch robot_name:="pioneer$i" robot_pose_x:="$(rosparam get /pioneer$i/x)" robot_pose_y:="$(rosparam get /pioneer$i/y)" robot_pose_yaw:="$(rosparam get /pioneer$i/a)" localization_type:="amcl" &
    pid="$pid $!"
    sleep 5s

    echo "# Launching move_base stack for 'pioneer$i'"
    roslaunch pioneer_nav2d move_base.launch robot_name:="pioneer$i" move_base_type:="move_base" base_global_planner:="NavfnROS" base_local_planner:="TebLocalPlannerROS" mcp_use:=true &
    pid="$pid $!"
    sleep 5s
done

echo "## Launching MultiAgent MPC..."
roslaunch pioneer_nav2d ma_diff_mpc.launch &
pid="$pid $!"
sleep 5s

echo "## Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:="MPC_usage_two_pioneers" &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
sleep 8h # A good 8h sleep
