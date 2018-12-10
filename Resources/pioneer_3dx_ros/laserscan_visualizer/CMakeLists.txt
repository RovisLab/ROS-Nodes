cmake_minimum_required(VERSION 2.8.3)
project(laserscan_visualizer)
find_package(OpenCV REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport message_filters std_msgs  nav_msgs sensor_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(listener_node src/listener_node.cpp)
target_link_libraries(listener_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(bind_exact src/bind_exact.cpp)
target_link_libraries(bind_exact ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(state_saver src/state_saver.cpp)
target_link_libraries(state_saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_executable(laserimage_node src/laserimage.cpp)
#target_link_libraries(laserimage_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(try src/try.cpp)
#target_link_libraries(try ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
