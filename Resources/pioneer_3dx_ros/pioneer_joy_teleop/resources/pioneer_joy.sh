#!/bin/bash
my_pid=$$
# Parse command-line options

# Option strings
ARGUMENT_LIST=(
    topic
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
   echo -e "\t--topic Command velocity topic name of the robot"
   exit 1 # Exit script after printing help
}

# extract options and their arguments into variables.
while [[ $# -gt 0 ]]; do
  case "$1" in
    --topic )
      topic="$2"
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
if [ -z "$topic" ]
then
  echo "";
  echo "Some or all of the parameters are empty";
  helpFunction
fi

echo "topic = $topic"

echo "Launching joy teleop..."
roslaunch pioneer_joy_teleop pioneer_joy_teleop.launch topic:="$topic" &
pid="$pid $!"
sleep 5s

echo "$pid" > /home/$(whoami)/script_pid_joy.txt

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
