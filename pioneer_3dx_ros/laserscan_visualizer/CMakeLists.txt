cmake_minimum_required(VERSION 2.8.3)
project(laserscan_visualizer)
find_package(OpenCV REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
