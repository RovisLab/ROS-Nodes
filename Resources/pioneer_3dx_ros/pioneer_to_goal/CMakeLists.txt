cmake_minimum_required(VERSION 2.8.3)
project(pioneer_to_goal)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  rospy
  tf
  darknet_ros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_multi
  CATKIN_DEPENDS actionlib darknet_ros_msgs move_base_msgs roscpp rospy tf
#  DEPENDS system_lib
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_navigation src/map_navigation.cpp)
add_executable(map_navigation_darknet src/map_navigation_darknet.cpp)
add_executable(send_one src/send_one.cpp)


add_dependencies(map_navigation_darknet darknet_ros_msgs_generate_messages_cpp)

target_link_libraries(map_navigation
  ${catkin_LIBRARIES}
)
target_link_libraries(map_navigation_darknet
  ${catkin_LIBRARIES}
)
target_link_libraries(send_one
  ${catkin_LIBRARIES}
)

