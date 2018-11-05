cmake_minimum_required(VERSION 2.8.3)
project(multi_nav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  rospy
  tf
  coords_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_multi
  CATKIN_DEPENDS actionlib move_base_msgs roscpp rospy tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(send_goal src/send_goal.cpp)
add_executable(send_multiple_goal src/send_multiple_goal.cpp)
add_executable(map_navigation src/map_navigation.cpp)

## Generate the msg before the node.
add_dependencies(send_goal coords_msgs_generate_messages_cpp)
add_dependencies(send_multiple_goal coords_msgs_generate_messages_cpp)

target_link_libraries(send_goal
  ${catkin_LIBRARIES}
)
target_link_libraries(send_multiple_goal
  ${catkin_LIBRARIES}
)
target_link_libraries(map_navigation
  ${catkin_LIBRARIES}
)


