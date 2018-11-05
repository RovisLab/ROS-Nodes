cmake_minimum_required(VERSION 2.8.3)
project(pioneer_nav2d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gmapping
  move_base
  amcl
  map_server
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

