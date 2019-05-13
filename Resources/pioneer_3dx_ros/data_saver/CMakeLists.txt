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

add_executable(multi_odom src/multi_odom.cpp)
target_link_libraries(multi_odom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(one_pioneer_path src/one_pioneer_path.cpp)
target_link_libraries(one_pioneer_path ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(two_pioneers_path src/two_pioneers_path.cpp)
target_link_libraries(two_pioneers_path ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
