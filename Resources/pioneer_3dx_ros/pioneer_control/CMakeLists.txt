cmake_minimum_required(VERSION 2.8.3)
project(pioneer_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rosconsole
  sensor_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
