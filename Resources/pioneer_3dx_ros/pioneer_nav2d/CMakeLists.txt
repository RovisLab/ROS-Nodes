cmake_minimum_required(VERSION 2.8.3)
project(pioneer_nav2d)
## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -O3)

# options for build configuration
option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF) 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gmapping
  move_base
  amcl
  map_server
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  ackermann_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS} include )
include_directories(/usr/include)
link_directories(/usr/lib)

# MPC_Node
ADD_EXECUTABLE( MPC_Node src/MPC.cpp src/MPC_Node.cpp )
TARGET_LINK_LIBRARIES(MPC_Node ipopt ${catkin_LIBRARIES} )

# Pure Pursuit Node
add_executable(Pure_Pursuit src/Pure_Pursuit.cpp)
target_link_libraries(Pure_Pursuit ${catkin_LIBRARIES})
