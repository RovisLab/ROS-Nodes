cmake_minimum_required(VERSION 2.8.3)
project(pioneer_double_nav)


find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pioneer_double_nav
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_transformer src/velocity_transformer.cpp)
target_link_libraries(velocity_transformer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(try src/try.cpp)
target_link_libraries(try ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

