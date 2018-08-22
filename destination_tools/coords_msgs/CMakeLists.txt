cmake_minimum_required(VERSION 2.8.3)
project(coords_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

add_message_files(DIRECTORY msg
	FILES
	Coords.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  DEPENDS message_runtime std_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

