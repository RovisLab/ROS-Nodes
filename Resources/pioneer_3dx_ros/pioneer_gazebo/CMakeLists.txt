cmake_minimum_required(VERSION 2.8.3)
project(pioneer_gazebo)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package()

add_library(animated_box SHARED src/animated_box.cc)
target_link_libraries(animated_box ${GAZEBO_LIBRARIES})

add_library(horizontal_move SHARED src/horizontal_move.cc)
target_link_libraries(horizontal_move ${GAZEBO_LIBRARIES})

add_library(horizontal_move2 SHARED src/horizontal_move2.cc)
target_link_libraries(horizontal_move2 ${GAZEBO_LIBRARIES})

add_library(vertical_move SHARED src/vertical_move.cc)
target_link_libraries(vertical_move ${GAZEBO_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
