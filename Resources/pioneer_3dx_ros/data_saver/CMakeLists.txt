cmake_minimum_required(VERSION 2.8.3)
project(data_saver)
find_package(OpenCV REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport message_filters std_msgs  nav_msgs sensor_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(state_saver src/state_saver.cpp)
target_link_libraries(state_saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

