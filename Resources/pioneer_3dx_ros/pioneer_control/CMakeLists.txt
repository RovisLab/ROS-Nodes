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

add_executable(demo_pioneer_joy2twist_pioneer_node src/joy2twist_pioneer.cpp)

target_link_libraries(demo_pioneer_joy2twist_pioneer_node ${catkin_LIBRARIES})

install(
  TARGETS 
    demo_pioneer_joy2twist_pioneer_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
