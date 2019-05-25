#!/bin/bash

# Parse command-line options

# Option strings
ARGUMENT_LIST=(
    pose_file
    robot_names_file
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
   echo -e "\t--world Name of the world used"
   echo -e "\t--robot_URDF_model Name of the URDF model of the robot"
   echo -e "\t--pose_file Pose file name"
   echo -e "\t--kinect_to_laserscan Option to use kinect"
   echo -e "\t--gmapping_config_type Gmapping configuration for the laser device"
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
    --kinect_to_laserscan )
      kinect_to_laserscan="$2"
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
if [ -z "$world" ] || [ -z "$robot_model" ] || [ -z "$pose_file" ] || [ -z "$kinect_to_laserscan" ] || [ -z "$gmapping_config_type" ]
then
  echo "";
  echo "Some or all of the parameters are empty";
  helpFunction
fi

echo "world = $world"
echo "robot_URDF_model = $robot_model"
echo "pose_file = $pose_file"
echo "kinect_to_laserscan = $kinect_to_laserscan"
echo "gmapping_config_type = $gmapping_config_type"

echo "Launching Gazebo..."
roslaunch pioneer_gazebo gazebo_world.launch world:="$world" &
pid="$pid $!"
sleep 5s

echo "Loading initialisation parameters..."
roslaunch pioneer_description pioneer_initialization.launch robot_URDF_model:="$robot_model" pose_file:="$pose_file" &
pid="$pid $!"
sleep 2s

echo "Launching Pioneers in Gazebo environment..."
roslaunch pioneer_description pioneer_description.launch robot_name:="pioneer1" robot_pose:="-x $(rosparam get /pioneer1/x) -y $(rosparam get /pioneer1/y) -Y $(rosparam get /pioneer1/a)" environment:="gazebo" use_real_kinect:=false kinect_to_laserscan:=$kinect_to_laserscan &
pid="$pid $!"
sleep 2s

echo "Launching SLAM Gmapping stack..."
roslaunch pioneer_nav2d gmapping_launcher.launch robot_name:="pioneer1" gmapping_config_type:="$gmapping_config_type" &
pid="$pid $!"
sleep 2s

echo "Launching rviz..."
roslaunch pioneer_description pioneer_visualization.launch rviz_config:="one_pioneer_mapping" &
pid="$pid $!"

echo "$pid" > /home/$(whoami)/script_pid.txt

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
