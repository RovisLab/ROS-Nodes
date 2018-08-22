cmake_minimum_required(VERSION 2.8.3)
project(coords_sender)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  coords_msgs

)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coords_sender
#  CATKIN_DEPENDS coords_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(goal_sender src/goal_sender.cpp)


target_link_libraries(goal_sender
  ${catkin_LIBRARIES}
)


