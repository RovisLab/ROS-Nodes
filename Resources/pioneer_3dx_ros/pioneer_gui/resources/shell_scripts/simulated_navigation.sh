#!/bin/bash

# Parse command-line options

# Option strings
ARGUMENT_LIST=(
    world
    robot_URDF_model
    pose_file
    participants
    kinect_to_laserscan
    localization_type
    base_global_planner
    base_local_planner
    rviz_config
)

# read the options
OPTS=$(getopt \
    --longoptions "$(printf "%s:," "${ARGUMENT_LIST[@]}")" \
    --name "$(basename "$0")" \
    --options "" \
    -- "$@"
)

if [ $? != 0 ] ; then echo "Failed to parse options...exiting." >&2 ; exit 1 ; fi

eval set -- "$OPTS"

# set initial values


helpFunction()
{
   echo ""
   echo "$0 [Options]:"
   echo -e "\t--world Name of the world used"
   echo -e "\t--robot_URDF_model Name of the URDF model of the robot"
   echo -e "\t--pose_file Pose file name"
   echo -e "\t--participants Number of the spawned robots"
   echo -e "\t--kinect_to_laserscan Option to use kinect"
   echo -e "\t--localization_type Type of localization algorithm used"
   echo -e "\t--base_global_planner Global navigation planner"
   echo -e "\t--base_local_planner Local navigation planner"
   echo -e "\t--rviz_config Global navigation planner"
   exit 1 # Exit script after printing help
}

# extract options and their arguments into variables.
while [[ $# -gt 0 ]]; do
  case "$1" in
    --world )
      world="$2"
      shift 2
      ;;
    --robot_URDF_model )
      robot_model="$2"
      shift 2
      ;;
    --pose_file )
      pose_file="$2"
      shift 2
      ;;
    --participants )
      participants="$2"
      shift 2
      ;;
    --kinect_to_laserscan )
      kinect_to_laserscan="$2"
      shift 2
      ;;
    --localization_type )
      localization_type="$2"
      shift 2
      ;;
    --base_global_planner )
      base_global_planner="$2"
      shift 2
      ;;
    --base_local_planner )
      base_local_planner="$2"
      shift 2
      ;;
    --rviz_config )
      rviz_config="$2"
      shift 2
      ;;
    -- )
      shift
      break
      ;;
    *)
      echo "Internal error!"
      exit 1
      ;;
  esac
done

# Print helpFunction in case parameters are empty
if [ -z "$world" ] || [ -z "$robot_model" ] || [ -z "$pose_file" ] || [ -z "$participants" ] || [ -z "$kinect_to_laserscan" ]  || [ -z "$localization_type" ] || [ -z "$base_global_planner" ] || [ -z "$base_local_planner" ] || [ -z "$rviz_config" ]
then
  echo "";
  echo "Some or all of the parameters are empty";
  helpFunction
fi

echo "world = $world"
echo "robot_URDF_model = $robot_model"
echo "pose_file = $pose_file"
echo "participants = $participants"
echo "kinect_to_laserscan = $kinect_to_laserscan"
echo "localization_type = $localization_type"
echo "base_global_planner = $base_global_planner"
echo "base_local_planner = $base_local_planner"
echo "rviz_config = $rviz_config"

echo "Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:="$world" &
pid="$pid $!"
sleep 5s

echo "Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch robot_URDF_model:="$robot_model" pose_file:="$pose_file" &
pid="$pid $!"
sleep 5s

echo "Launching Pioneers in Gazebo world..."
for i in `seq 1 $participants`;
do
  roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer$i" robot_pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" environment:="gazebo" use_real_kinect:=false kinect_to_laserscan:=$kinect_to_laserscan &
  pid="$pid $!"
  sleep 7s
done

echo "Launching map server..."
roslaunch pioneer_nav2d map_server.launch map_name:="$world" &
pid="$pid $!"
sleep 7s

echo "Launching localisation stack..."
for i in `seq 1 $participants`;
do
  roslaunch pioneer_nav2d localisation_launcher.launch robot_name:="pioneer$i" robot_pose_x:="$(rosparam get /pioneer$i/x)" robot_pose_y:="$(rosparam get /pioneer$i/y)" robot_pose_yaw:="$(rosparam get /pioneer$i/a)" localization_type:="$localization_type" &
  pid="$pid $!"
  sleep 7s
done

echo "Launching move_base stack..."
for i in `seq 1 $participants`;
do
  roslaunch pioneer_nav2d move_base.launch robot_name:="pioneer$i" move_base_type:="move_base" base_global_planner:="$base_global_planner" base_local_planner:="$base_local_planner" mcp_use:=false &
  pid="$pid $!"
  sleep 7s
done

echo "Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:="$rviz_config" &
pid="$pid $!"

echo "$pid" > /home/$(whoami)/script_pid.txt

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
