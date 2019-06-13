#!/bin/bash

# Parse command-line options

# Option strings
ARGUMENT_LIST=(
    pose_file
    real_robots_file
    gmapping_config_type
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
   echo -e "\t--pose_file Pose file name"
   echo -e "\t--real_robots_file The file containing the username and the address of the robobot"
   echo -e "\t--gmapping_config_type Gmapping configuration for the laser device"
   exit 1 # Exit script after printing help
}

# extract options and their arguments into variables.
while [[ $# -gt 0 ]]; do
  case "$1" in
    --pose_file )
      pose_file="$2"
      shift 2
      ;;
    --real_robots_file )
      real_robots_file="$2"
      shift 2
      ;;
    --gmapping_config_type )
      gmapping_config_type="$2"
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
if [ -z "$pose_file" ] || [ -z "$real_robots_file" ] || [ -z "$gmapping_config_type" ]
then
  echo "";
  echo "Some or all of the parameters are empty";
  helpFunction
fi

echo "pose_file = $pose_file"
echo "real_robots_file = $real_robots_file"
echo "gmapping_config_type = $gmapping_config_type"

echo "Launching pioneer_description..."
roslaunch pioneer_description pioneer_initialization.launch robot_URDF_model:="pioneer_kinect_real" pose_file:="$pose_file" real_robots_file:="$real_robots_file" &
pid="$pid $!"
sleep 10s

echo "Launching RosAria stack..."
roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer1" robot_pose:="-x $(rosparam get /pioneer1/x) -y $(rosparam get /pioneer1/y) -Y $(rosparam get /pioneer1/a)" environment:="real_world" use_real_kinect:=true kinect_to_laserscan:=true real_robot_username:="$(rosparam get /pioneer1/username)" real_robot_address:="$(rosparam get /pioneer1/address)" connect_robot_pc:=true  &
pid="$pid $!"
sleep 10s

echo "Launching SLAM Gmapping stack..."
roslaunch pioneer_nav2d gmapping_launcher.launch robot_name:="pioneer1" connect_robot_pc:=true gmapping_config_type:="$gmapping_config_type" real_robot_username:="$(rosparam get /pioneer1/username)" real_robot_address:="$(rosparam get /pioneer1/address)" &
pid="$pid $!"
sleep 10s

echo "Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:="one_pioneer_mapping" &
pid="$pid $!"
sleep 5s

echo "$pid" > /home/$(whoami)/script_pid.txt

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
